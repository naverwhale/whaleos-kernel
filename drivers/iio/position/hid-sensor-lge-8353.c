// SPDX-License-Identifier: GPL-2.0-only
/*
 * HID Sensors Driver
 * Copyright (c) 2020, Intel Corporation.
 * Copyright (c) 2022, LG Electronics.
 * Copyright (c) 2022, NAVER Corp.
 */
#include <linux/hid-sensor-hub.h>
#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/buffer.h>
#include <linux/platform_device.h>
#include <linux/module.h>

#include "../common/hid-sensors/hid-sensor-trigger.h"

enum lidangle_channel {
	CHANNEL_SCAN_INDEX_LID_ANGLE,
	CHANNEL_SCAN_INDEX_MAX,
};

static const char *const lid_angle_labels[CHANNEL_SCAN_INDEX_MAX] =
    { "lidangle", };

struct lid_angle_state {
	struct iio_dev *indio_dev;
	struct hid_sensor_hub_attribute_info lid_angle[CHANNEL_SCAN_INDEX_MAX];
	struct hid_sensor_hub_callbacks callbacks;
	struct hid_sensor_common common_attributes;
	const char *labels[CHANNEL_SCAN_INDEX_MAX];
	struct {
		u32 lid_angle_val[3];
		u64 timestamp __aligned(8);
	} scan;

	int tablet;
	struct input_dev *idev;

	int scale_pre_decml;
	int scale_post_decml;
	int scale_precision;
	int value_offset;
	u64 timestamp;
};

static const u32 lid_angle_addresses[CHANNEL_SCAN_INDEX_MAX] = {
	HID_USAGE_SENSOR_DATA_FIELD_CUSTOM_VALUE(6)
};

static const struct iio_chan_spec lid_angle_channels[] = {
	{
	 .type = IIO_ANGL,
	 .indexed = 1,
	 .channel = 0,
	 .info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
	 .info_mask_shared_by_type =
	 BIT(IIO_CHAN_INFO_OFFSET) |
	 BIT(IIO_CHAN_INFO_SCALE) |
	 BIT(IIO_CHAN_INFO_SAMP_FREQ) | BIT(IIO_CHAN_INFO_HYSTERESIS),
	 .scan_index = CHANNEL_SCAN_INDEX_LID_ANGLE,
	  },
	IIO_CHAN_SOFT_TIMESTAMP(CHANNEL_SCAN_INDEX_MAX)
};

/* Adjust channel real bits based on report descriptor */
static void lid_angle_adjust_channel_bit_mask(struct iio_chan_spec *channels,
					      int channel, int size)
{
	channels[channel].scan_type.sign = 's';
	/* Real storage bits will change based on the report desc. */
	channels[channel].scan_type.realbits = size * 8;
	/* Maximum size of a sample to capture is u32 */
	channels[channel].scan_type.storagebits = sizeof(u32) * 8;
}

/* Channel read_raw handler */
static int lid_angle_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan, int *val,
			      int *val2, long mask)
{
	struct lid_angle_state *lid_state = iio_priv(indio_dev);
	struct hid_sensor_hub_device *hsdev;
	int report_id = -1;
	int ret_type;
	s32 min;

	*val = 0;
	*val2 = 0;

	hsdev = lid_state->common_attributes.hsdev;
	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		hid_sensor_power_state(&lid_state->common_attributes, true);
		report_id = lid_state->lid_angle[chan->scan_index].report_id;
		min = lid_state->lid_angle[chan->scan_index].logical_minimum;
		if (report_id < 0) {
			hid_sensor_power_state(&lid_state->common_attributes,
					       false);
			ret_type = -EINVAL;
		} else {
			*val =
			    sensor_hub_input_attr_get_raw_value(lid_state->
								common_attributes.
								hsdev,
								hsdev->usage,
								lid_angle_addresses
								[chan->
								 scan_index],
								report_id,
								SENSOR_HUB_SYNC,
								min < 0) / 10;

			hid_sensor_power_state(&lid_state->common_attributes,
					       false);
			ret_type = IIO_VAL_INT;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		*val = lid_state->scale_pre_decml;
		*val2 = lid_state->scale_post_decml;
		ret_type = lid_state->scale_precision;
		break;
	case IIO_CHAN_INFO_OFFSET:
		*val = lid_state->value_offset;
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret_type =
		    hid_sensor_read_samp_freq_value(&lid_state->
						    common_attributes, val,
						    val2);
		break;
	case IIO_CHAN_INFO_HYSTERESIS:
		ret_type =
		    hid_sensor_read_raw_hyst_value(&lid_state->
						   common_attributes, val,
						   val2);
		break;
	default:
		ret_type = -EINVAL;
	}
	return ret_type;
}

/* Channel write_raw handler */
static int lid_angle_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan, int val,
			       int val2, long mask)
{
	struct lid_angle_state *lid_state = iio_priv(indio_dev);
	int ret = 0;

	switch (mask) {
	case IIO_CHAN_INFO_SAMP_FREQ:
		ret =
		    hid_sensor_write_samp_freq_value(&lid_state->
						     common_attributes, val,
						     val2);
		break;
	case IIO_CHAN_INFO_HYSTERESIS:
		ret =
		    hid_sensor_write_raw_hyst_value(&lid_state->
						    common_attributes, val,
						    val2);
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static const struct iio_info lid_angle_info = {
	.read_raw = lid_angle_read_raw,
	.write_raw = lid_angle_write_raw,
};

/* Callback handler to send event after all samples are received and captured */
static int lid_angle_proc_event(struct hid_sensor_hub_device *hsdev,
				unsigned int usage_id, void *priv)
{
	struct iio_dev *indio_dev = platform_get_drvdata(priv);
	struct lid_angle_state *lid_state = iio_priv(indio_dev);

	if (atomic_read(&lid_state->common_attributes.data_ready)) {
		if (!lid_state->timestamp)
			lid_state->timestamp = iio_get_time_ns(indio_dev);

		iio_push_to_buffers_with_timestamp(indio_dev,
						   &lid_state->scan,
						   lid_state->timestamp);
		lid_state->timestamp = 0;
	}
	return 0;
}

/* Capture samples in local storage */
static int lid_angle_capture_sample(struct hid_sensor_hub_device *hsdev,
				    unsigned int usage_id, size_t raw_len,
				    char *raw_data, void *priv)
{
	struct iio_dev *indio_dev = platform_get_drvdata(priv);
	struct lid_angle_state *lid_state = iio_priv(indio_dev);
	int offset;
	int ret = -EINVAL;

	switch (usage_id) {
	case HID_USAGE_SENSOR_DATA_FIELD_CUSTOM_VALUE(6):
		offset = usage_id - HID_USAGE_SENSOR_DATA_FIELD_CUSTOM_VALUE(6);
		lid_state->scan.lid_angle_val[offset] = *(u32 *) raw_data;
		// https://source.chromium.org/chromiumos/chromiumos/codesearch/+/main:src/platform/ec/common/motion_lid.c
		if ((!lid_state->tablet && *(u16 *) raw_data > 2000)
		    || (lid_state->tablet && *(u16 *) raw_data < 1600
			&& *(u16 *) raw_data > 100)) {
			dev_dbg(indio_dev->dev.parent, "angle %d, send event\n",
				*(u16 *) raw_data / 10);
			lid_state->tablet = !lid_state->tablet;
			input_report_switch(lid_state->idev, SW_TABLET_MODE,
					    lid_state->tablet);
			input_sync(lid_state->idev);
		}
		return 0;
	case HID_USAGE_SENSOR_TIME_TIMESTAMP:
		lid_state->timestamp =
		    hid_sensor_convert_timestamp(&lid_state->common_attributes,
						 *(int64_t *) raw_data);
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

static int lid_angle_parse_report(struct platform_device *pdev,
				  struct hid_sensor_hub_device *hsdev,
				  struct iio_chan_spec *channels,
				  unsigned int usage_id,
				  struct lid_angle_state *lid_state)
{
	int ret;
	int i;

	for (i = 0; i < CHANNEL_SCAN_INDEX_MAX; ++i) {
		ret = sensor_hub_input_get_attribute_info(hsdev,
							  HID_INPUT_REPORT,
							  usage_id,
							  lid_angle_addresses
							  [i],
							  &lid_state->
							  lid_angle[i]);
		if (ret < 0)
			return ret;
		lid_angle_adjust_channel_bit_mask(channels, i,
						  lid_state->lid_angle[i].size);
	}
	lid_state->scale_precision =
	    hid_sensor_format_scale(HID_USAGE_SENSOR_HINGE,
				    &lid_state->
				    lid_angle[CHANNEL_SCAN_INDEX_LID_ANGLE],
				    &lid_state->scale_pre_decml,
				    &lid_state->scale_post_decml);

	return ret;
}

/* Function to initialize the processing for usage id */
static int lid_angle_probe(struct platform_device *pdev)
{
	int l, ret = 0;
	static const char *name = "angl";
	struct iio_dev *indio_dev;
	struct lid_angle_state *lid_state;
	struct hid_sensor_hub_device *hsdev = pdev->dev.platform_data;
	struct input_dev *idev;

	indio_dev = devm_iio_device_alloc(&pdev->dev, sizeof(*lid_state));
	if (!indio_dev)
		return -ENOMEM;

	platform_set_drvdata(pdev, indio_dev);

	lid_state = iio_priv(indio_dev);
	lid_state->common_attributes.hsdev = hsdev;
	lid_state->common_attributes.pdev = pdev;
	lid_state->indio_dev = indio_dev;
	for (l = 0; l < CHANNEL_SCAN_INDEX_MAX; l++)
		lid_state->labels[l] = lid_angle_labels[l];

	ret = hid_sensor_parse_common_attributes(hsdev,
						 hsdev->usage,
						 &lid_state->common_attributes,
						 lid_angle_addresses,
						 ARRAY_SIZE
						 (lid_angle_addresses));
	if (ret) {
		dev_err(&pdev->dev, "failed to setup common attributes\n");
		return ret;
	}

	indio_dev->channels = devm_kmemdup(&indio_dev->dev, lid_angle_channels,
					   sizeof(lid_angle_channels),
					   GFP_KERNEL);
	if (!indio_dev->channels) {
		dev_err(&pdev->dev, "failed to duplicate channels\n");
		return -ENOMEM;
	}

	ret = lid_angle_parse_report(pdev, hsdev,
				     (struct iio_chan_spec *)indio_dev->
				     channels, hsdev->usage, lid_state);
	if (ret) {
		dev_err(&pdev->dev, "failed to setup attributes\n");
		goto error_free_dev_mem;
	}

	indio_dev->num_channels = ARRAY_SIZE(lid_angle_channels);
	indio_dev->info = &lid_angle_info;
	indio_dev->name = name;
	indio_dev->modes = INDIO_DIRECT_MODE;

	atomic_set(&lid_state->common_attributes.data_ready, 0);

	ret = hid_sensor_setup_trigger(indio_dev, name,
				       &lid_state->common_attributes);
	if (ret < 0) {
		dev_err(&pdev->dev, "trigger setup failed\n");
		goto error_free_dev_mem;
	}

	ret = iio_device_register(indio_dev);
	if (ret) {
		dev_err(&pdev->dev, "device register failed\n");
		goto error_remove_trigger;
	}

	lid_state->callbacks.send_event = lid_angle_proc_event;
	lid_state->callbacks.capture_sample = lid_angle_capture_sample;
	lid_state->callbacks.pdev = pdev;
	ret = sensor_hub_register_callback(hsdev, hsdev->usage,
					   &lid_state->callbacks);
	if (ret < 0) {
		dev_err(&pdev->dev, "callback reg failed\n");
		goto error_iio_unreg;
	}

	idev = input_allocate_device();
	if (!idev)
		return -ENOMEM;

	idev->name = "tablet_mode";
	idev->id.bustype = BUS_HOST;
	input_set_capability(idev, EV_SW, SW_TABLET_MODE);

	ret = input_register_device(idev);
	if (ret) {
		input_free_device(idev);
		return ret;
	}

	lid_state->idev = idev;

	return ret;

error_iio_unreg:
	iio_device_unregister(indio_dev);
error_remove_trigger:
	hid_sensor_remove_trigger(indio_dev, &lid_state->common_attributes);
error_free_dev_mem:
	kfree(indio_dev->channels);
	return ret;
}

/* Function to deinitialize the processing for usage id */
static int lid_angle_remove(struct platform_device *pdev)
{
	struct hid_sensor_hub_device *hsdev = pdev->dev.platform_data;
	struct iio_dev *indio_dev = platform_get_drvdata(pdev);
	struct lid_angle_state *lid_state = iio_priv(indio_dev);

	sensor_hub_remove_callback(hsdev, hsdev->usage);
	iio_device_unregister(indio_dev);
	hid_sensor_remove_trigger(indio_dev, &lid_state->common_attributes);
	kfree(indio_dev->channels);

	return 0;
}

static const struct platform_device_id lid_angle_ids[] = {
	{
	 /* Format: HID-SENSOR-usage_id_in_hex_lowercase */
	 .name = "HID-SENSOR-LGE-8353",
	  },
	{ /* sentinel */  }
};

MODULE_DEVICE_TABLE(platform, lid_angle_ids);

static struct platform_driver lid_angle_platform_driver = {
	.id_table = lid_angle_ids,
	.driver = {
		   .name = KBUILD_MODNAME,
		   .pm = &hid_sensor_pm_ops,
		    },
	.probe = lid_angle_probe,
	.remove = lid_angle_remove,
};

module_platform_driver(lid_angle_platform_driver);

MODULE_DESCRIPTION("LGE 8353 HID Sensor");
MODULE_AUTHOR("LGE");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS(IIO_HID);
