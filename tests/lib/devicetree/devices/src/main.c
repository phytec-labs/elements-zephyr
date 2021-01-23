/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <ztest.h>
#include <devicetree.h>
#include <device.h>
#include <drivers/gpio.h>

#define TEST_GPIO DT_NODELABEL(test_gpio_0)
#define TEST_I2C DT_NODELABEL(test_i2c)
#define TEST_DEVA DT_NODELABEL(test_dev_a)
#define TEST_GPIOX DT_NODELABEL(test_gpiox)
#define TEST_DEVB DT_NODELABEL(test_dev_b)

static const struct device *devlist;
static const struct device *devlist_end;

static device_handle_t init_order[10];

static int dev_init(const struct device *dev)
{
	static uint8_t init_idx;

	__ASSERT_NO_MSG(init_idx < ARRAY_SIZE(init_order));
	init_order[init_idx++] = device_handle_get(dev);

	return 0;
}

DEVICE_DT_DEFINE(TEST_GPIO, dev_init, device_pm_control_nop,
		 NULL, NULL, PRE_KERNEL_1, 90, NULL);
DEVICE_DT_DEFINE(TEST_I2C, dev_init, device_pm_control_nop,
		 NULL, NULL, POST_KERNEL, 10, NULL);
DEVICE_DT_DEFINE(TEST_DEVA, dev_init, device_pm_control_nop,
		 NULL, NULL, POST_KERNEL, 20, NULL);
/* NB: Intentional init devb before required gpiox */
DEVICE_DT_DEFINE(TEST_DEVB, dev_init, device_pm_control_nop,
		 NULL, NULL, POST_KERNEL, 30, NULL);
DEVICE_DT_DEFINE(TEST_GPIOX, dev_init, device_pm_control_nop,
		 NULL, NULL, POST_KERNEL, 40, NULL);

#define DEV_HDL(node_id) device_handle_get(DEVICE_DT_GET(node_id))

static void test_init_order(void)
{
	zassert_equal(init_order[0], DEV_HDL(TEST_GPIO),
		      NULL);
	zassert_equal(init_order[1], DEV_HDL(TEST_I2C),
		      NULL);
	zassert_equal(init_order[2], DEV_HDL(TEST_DEVA),
		      NULL);
	zassert_equal(init_order[3], DEV_HDL(TEST_DEVB),
		      NULL);
	zassert_equal(init_order[4], DEV_HDL(TEST_GPIOX),
		      NULL);
}

static bool check_handle(device_handle_t hdl,
			 const device_handle_t *hdls,
			 size_t nhdls)
{
	const device_handle_t *hdle = hdls + nhdls;

	while (hdls < hdle) {
		if (*hdls == hdl) {
			return true;
		}
		++hdls;
	}

	return false;
}

static void test_requires(void)
{
	size_t nhdls = 0;
	const device_handle_t *hdls;
	const struct device *dev;

	/* TEST_GPIO: no req */
	dev = device_get_binding(DT_LABEL(TEST_GPIO));
	zassert_equal(dev, DEVICE_DT_GET(TEST_GPIO), NULL);
	hdls = device_get_requires_handles(dev, &nhdls);
	zassert_equal(nhdls, 0, NULL);

	/* TEST_I2C: no req */
	dev = device_get_binding(DT_LABEL(TEST_I2C));
	zassert_equal(dev, DEVICE_DT_GET(TEST_I2C), NULL);
	hdls = device_get_requires_handles(dev, &nhdls);
	zassert_equal(nhdls, 0, NULL);

	/* TEST_DEVA: TEST_I2C GPIO */
	dev = device_get_binding(DT_LABEL(TEST_DEVA));
	zassert_equal(dev, DEVICE_DT_GET(TEST_DEVA), NULL);
	hdls = device_get_requires_handles(dev, &nhdls);
	zassert_equal(nhdls, 2, NULL);
	zassert_true(check_handle(DEV_HDL(TEST_I2C), hdls, nhdls), NULL);
	zassert_true(check_handle(DEV_HDL(TEST_GPIO), hdls, nhdls), NULL);

	/* TEST_GPIOX: TEST_I2C */
	dev = device_get_binding(DT_LABEL(TEST_GPIOX));
	zassert_equal(dev, DEVICE_DT_GET(TEST_GPIOX), NULL);
	hdls = device_get_requires_handles(dev, &nhdls);
	zassert_equal(nhdls, 1, NULL);
	zassert_true(check_handle(DEV_HDL(TEST_I2C), hdls, nhdls), NULL);

	/* TEST_DEVB: TEST_I2C TEST_GPIOX */
	dev = device_get_binding(DT_LABEL(TEST_DEVB));
	zassert_equal(dev, DEVICE_DT_GET(TEST_DEVB), NULL);
	hdls = device_get_requires_handles(dev, &nhdls);
	zassert_equal(nhdls, 2, NULL);
	zassert_true(check_handle(DEV_HDL(TEST_I2C), hdls, nhdls), NULL);
	zassert_true(check_handle(DEV_HDL(TEST_GPIOX), hdls, nhdls), NULL);
}

void test_main(void)
{
	size_t ndevs;

	ndevs = z_device_get_all_static(&devlist);
	devlist_end = devlist + ndevs;

	ztest_test_suite(devicetree_driver,
			 ztest_unit_test(test_init_order),
			 ztest_unit_test(test_requires)
		);
	ztest_run_test_suite(devicetree_driver);
}
