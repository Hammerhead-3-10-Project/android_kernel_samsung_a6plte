/*
 * Samsung 3.5mm headset jack detection (Galaxy A6+).
 *
 * The plug is seen on a TLMM GPIO; the type (3 pole headphones / 4 pole
 * headset) comes from the voltage on the microphone line, read with the PMIC
 * VADC while the microphone bias is on. A second GPIO signals the headset
 * button, whose function is again told apart by the ADC voltage.
 *
 * Reimplementation for 4.9 of the Samsung 3.18 driver (GPL v2): same device
 * tree data (zones), but input events and the h2w switch are reported
 * directly instead of through gpio_event.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/qpnp/qpnp-adc.h>
#include <linux/sec_jack.h>
#include <linux/switch.h>
#include <linux/workqueue.h>

#define NUM_ZONES		4
#define DEFAULT_DET_DB_MS	100
#define DEFAULT_KEY_DB_MS	30
#define WAKE_TIME_MS		5000
#define KEY_ENABLE_DELAY_MS	1000

struct det_zone {
	u32 adc_high;
	u32 delay_us;
	u32 check_count;
	u32 jack_type;
};

struct but_zone {
	u32 code;
	u32 adc_low;
	u32 adc_high;
};

struct sec_jack {
	struct device *dev;
	struct input_dev *input;
	struct qpnp_vadc_chip *vadc;
	struct workqueue_struct *wq;
	struct delayed_work detect_work;
	struct delayed_work key_work;
	struct delayed_work key_enable_work;
	struct wakeup_source ws;
	struct det_zone det_zones[NUM_ZONES];
	struct but_zone but_zones[NUM_ZONES];
	int det_gpio, key_gpio;
	int det_irq, key_irq;
	u32 det_db_ms, key_db_ms;
	u32 adc_channel;
	bool det_active_high;
	bool key_active_high;
	bool keys_enabled;
	unsigned int pressed_code;
	unsigned int cur_type;
};

static struct switch_dev h2w = { .name = "h2w" };
static struct switch_dev send_end = { .name = "send_end" };

static bool jack_inserted(struct sec_jack *j)
{
	return !!gpio_get_value(j->det_gpio) == j->det_active_high;
}

/* returns the microphone line voltage in mV, or a negative errno */
static int jack_read_adc(struct sec_jack *j)
{
	struct qpnp_vadc_result res;
	int rc;

	if (IS_ERR_OR_NULL(j->vadc)) {
		j->vadc = qpnp_get_vadc(j->dev, "earjack-read");
		if (IS_ERR(j->vadc)) {
			rc = PTR_ERR(j->vadc);
			j->vadc = NULL;
			return rc;
		}
	}

	rc = qpnp_vadc_read(j->vadc, j->adc_channel, &res);
	if (rc < 0)
		return rc;

	return div_s64(res.physical, 1000);
}

static void jack_set_type(struct sec_jack *j, unsigned int type)
{
	bool four_pole = type == SEC_HEADSET_4POLE ||
			 type == SEC_EXTERNAL_ANTENNA;

	if (type == j->cur_type) {
		if (!four_pole)
			msm_anlg_cdc_sec_micbias(false);
		return;
	}

	if (four_pole) {
		/* the button line is only meaningful once the bias has settled */
		queue_delayed_work(j->wq, &j->key_enable_work,
				   msecs_to_jiffies(KEY_ENABLE_DELAY_MS));
	} else {
		msm_anlg_cdc_sec_micbias(false);
		cancel_delayed_work_sync(&j->key_enable_work);
		j->keys_enabled = false;
	}

	j->cur_type = type;
	dev_info(j->dev, "jack type: %u\n", type);
	switch_set_state(&h2w, type);
}

static void jack_not_inserted(struct sec_jack *j)
{
	jack_set_type(j, SEC_JACK_NO_DEVICE);
}

static void jack_determine_type(struct sec_jack *j)
{
	unsigned int count[NUM_ZONES] = { 0 };
	int adc, i;

	msm_anlg_cdc_sec_micbias(true);

	while (jack_inserted(j)) {
		adc = jack_read_adc(j);
		if (adc < 0) {
			dev_err(j->dev, "cannot read the ADC: %d\n", adc);
			return;
		}
		dev_dbg(j->dev, "mic line %d mV\n", adc);

		for (i = 0; i < NUM_ZONES; i++) {
			if (adc <= j->det_zones[i].adc_high) {
				if (++count[i] > j->det_zones[i].check_count) {
					dev_info(j->dev, "mic line %d mV\n", adc);
					jack_set_type(j, j->det_zones[i].jack_type);
					return;
				}
				if (j->det_zones[i].delay_us)
					usleep_range(j->det_zones[i].delay_us,
						     j->det_zones[i].delay_us + 500);
				break;
			}
		}
		if (i == NUM_ZONES)
			return;
	}

	dev_info(j->dev, "jack removed before the detection completed\n");
	jack_not_inserted(j);
}

static void jack_detect_work(struct work_struct *work)
{
	struct sec_jack *j = container_of(work, struct sec_jack,
					  detect_work.work);

	__pm_wakeup_event(&j->ws, WAKE_TIME_MS);

	/* debounce: the plug must be seen for the whole period */
	msleep(j->det_db_ms);
	if (!jack_inserted(j)) {
		jack_not_inserted(j);
		return;
	}

	jack_determine_type(j);
}

static irqreturn_t jack_detect_irq(int irq, void *data)
{
	struct sec_jack *j = data;

	mod_delayed_work(j->wq, &j->detect_work, 0);

	return IRQ_HANDLED;
}

static void jack_key_enable_work(struct work_struct *work)
{
	struct sec_jack *j = container_of(work, struct sec_jack,
					  key_enable_work.work);

	j->keys_enabled = true;
}

static void jack_key_work(struct work_struct *work)
{
	struct sec_jack *j = container_of(work, struct sec_jack,
					  key_work.work);
	bool pressed = !!gpio_get_value(j->key_gpio) == j->key_active_high;
	int adc, i;

	if (!j->keys_enabled)
		return;

	__pm_wakeup_event(&j->ws, WAKE_TIME_MS);

	if (!pressed) {
		if (j->pressed_code) {
			input_report_key(j->input, j->pressed_code, 0);
			input_sync(j->input);
			switch_set_state(&send_end, 0);
			dev_info(j->dev, "key %u released\n", j->pressed_code);
			j->pressed_code = 0;
		}
		return;
	}

	adc = jack_read_adc(j);
	if (adc < 0)
		return;

	for (i = 0; i < NUM_ZONES; i++) {
		if (adc >= j->but_zones[i].adc_low &&
		    adc <= j->but_zones[i].adc_high) {
			j->pressed_code = j->but_zones[i].code;
			input_report_key(j->input, j->pressed_code, 1);
			input_sync(j->input);
			switch_set_state(&send_end, 1);
			dev_info(j->dev, "adc %d mV: key %u pressed\n",
				 adc, j->pressed_code);
			return;
		}
	}

	dev_warn(j->dev, "button ignored, adc %d mV\n", adc);
}

static irqreturn_t jack_key_irq(int irq, void *data)
{
	struct sec_jack *j = data;

	mod_delayed_work(j->wq, &j->key_work, msecs_to_jiffies(j->key_db_ms));

	return IRQ_HANDLED;
}

static int jack_parse_dt(struct sec_jack *j)
{
	struct device_node *np = j->dev->of_node;
	u32 det[NUM_ZONES * 4], but[NUM_ZONES * 3];
	int i, rc;

	j->det_gpio = of_get_named_gpio(np, "detect-gpio", 0);
	j->key_gpio = of_get_named_gpio(np, "key-gpio", 0);
	if (!gpio_is_valid(j->det_gpio) || !gpio_is_valid(j->key_gpio)) {
		dev_err(j->dev, "detect-gpio / key-gpio missing\n");
		return -EINVAL;
	}

	j->det_db_ms = DEFAULT_DET_DB_MS;
	j->key_db_ms = DEFAULT_KEY_DB_MS;
	of_property_read_u32(np, "det-dbtime", &j->det_db_ms);
	of_property_read_u32(np, "key-dbtime", &j->key_db_ms);
	j->det_active_high = of_property_read_bool(np, "det-active-high");
	j->key_active_high = of_property_read_bool(np, "send-end-active-high");

	rc = of_property_read_u32(np, "earjack-adc-channel", &j->adc_channel);
	if (rc)
		return rc;

	rc = of_property_read_u32_array(np, "det-zones", det, ARRAY_SIZE(det));
	if (rc)
		return rc;
	rc = of_property_read_u32_array(np, "but-zones", but, ARRAY_SIZE(but));
	if (rc)
		return rc;

	for (i = 0; i < NUM_ZONES; i++) {
		j->det_zones[i].adc_high = det[i * 4];
		j->det_zones[i].delay_us = det[i * 4 + 1];
		j->det_zones[i].check_count = det[i * 4 + 2];
		j->det_zones[i].jack_type = det[i * 4 + 3];
		j->but_zones[i].code = but[i * 3];
		j->but_zones[i].adc_low = but[i * 3 + 1];
		j->but_zones[i].adc_high = but[i * 3 + 2];
	}

	return 0;
}

static int sec_jack_probe(struct platform_device *pdev)
{
	struct sec_jack *j;
	struct pinctrl *pc;
	int i, rc;

	/* the microphone bias lives in the codec: wait for the sound card */
	rc = msm_anlg_cdc_sec_micbias(false);
	if (rc == -ENODEV)
		return -EPROBE_DEFER;

	j = devm_kzalloc(&pdev->dev, sizeof(*j), GFP_KERNEL);
	if (!j)
		return -ENOMEM;
	j->dev = &pdev->dev;

	rc = jack_parse_dt(j);
	if (rc)
		return rc;

	pc = devm_pinctrl_get_select(&pdev->dev, "earjack_gpio_active");
	if (IS_ERR(pc) && PTR_ERR(pc) == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	rc = devm_gpio_request_one(&pdev->dev, j->det_gpio, GPIOF_IN,
				   "jack_detect");
	if (rc)
		return rc;
	rc = devm_gpio_request_one(&pdev->dev, j->key_gpio, GPIOF_IN,
				   "jack_key");
	if (rc)
		return rc;

	j->input = devm_input_allocate_device(&pdev->dev);
	if (!j->input)
		return -ENOMEM;
	j->input->name = "sec_jack";
	j->input->id.bustype = BUS_HOST;
	for (i = 0; i < NUM_ZONES; i++)
		input_set_capability(j->input, EV_KEY, j->but_zones[i].code);
	rc = input_register_device(j->input);
	if (rc)
		return rc;

	rc = switch_dev_register(&h2w);
	if (rc)
		return rc;
	rc = switch_dev_register(&send_end);
	if (rc)
		goto err_h2w;

	wakeup_source_init(&j->ws, "sec_jack");
	j->wq = create_singlethread_workqueue("sec_jack");
	if (!j->wq) {
		rc = -ENOMEM;
		goto err_ws;
	}
	INIT_DELAYED_WORK(&j->detect_work, jack_detect_work);
	INIT_DELAYED_WORK(&j->key_work, jack_key_work);
	INIT_DELAYED_WORK(&j->key_enable_work, jack_key_enable_work);
	platform_set_drvdata(pdev, j);

	j->det_irq = gpio_to_irq(j->det_gpio);
	j->key_irq = gpio_to_irq(j->key_gpio);

	rc = devm_request_threaded_irq(&pdev->dev, j->det_irq, NULL,
				       jack_detect_irq,
				       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				       IRQF_ONESHOT, "sec_jack_detect", j);
	if (rc)
		goto err_wq;
	rc = devm_request_threaded_irq(&pdev->dev, j->key_irq, NULL,
				       jack_key_irq,
				       IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING |
				       IRQF_ONESHOT, "sec_jack_key", j);
	if (rc)
		goto err_wq;
	enable_irq_wake(j->det_irq);

	/* a headset may already be plugged in */
	queue_delayed_work(j->wq, &j->detect_work, 0);

	dev_info(&pdev->dev, "registered (detect gpio %d, key gpio %d)\n",
		 j->det_gpio, j->key_gpio);
	return 0;

err_wq:
	destroy_workqueue(j->wq);
err_ws:
	wakeup_source_trash(&j->ws);
	switch_dev_unregister(&send_end);
err_h2w:
	switch_dev_unregister(&h2w);
	return rc;
}

static int sec_jack_remove(struct platform_device *pdev)
{
	struct sec_jack *j = platform_get_drvdata(pdev);

	disable_irq_wake(j->det_irq);
	devm_free_irq(&pdev->dev, j->det_irq, j);
	devm_free_irq(&pdev->dev, j->key_irq, j);
	cancel_delayed_work_sync(&j->detect_work);
	cancel_delayed_work_sync(&j->key_work);
	cancel_delayed_work_sync(&j->key_enable_work);
	destroy_workqueue(j->wq);
	wakeup_source_trash(&j->ws);
	switch_dev_unregister(&send_end);
	switch_dev_unregister(&h2w);
	msm_anlg_cdc_sec_micbias(false);

	return 0;
}

static const struct of_device_id sec_jack_dt_match[] = {
	{ .compatible = "sec_jack" },
	{ }
};
MODULE_DEVICE_TABLE(of, sec_jack_dt_match);

static struct platform_driver sec_jack_driver = {
	.probe = sec_jack_probe,
	.remove = sec_jack_remove,
	.driver = {
		.name = "sec_jack",
		.of_match_table = sec_jack_dt_match,
	},
};
module_platform_driver(sec_jack_driver);

MODULE_DESCRIPTION("Samsung headset jack detection");
MODULE_LICENSE("GPL v2");
