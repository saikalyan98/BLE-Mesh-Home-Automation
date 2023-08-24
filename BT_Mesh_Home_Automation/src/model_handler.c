/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *	Modified By: M. Sai Kalyan.
 *	Date: 24-08-2023
 *	Project: Home Automation with 8 Elements. Among the 8, 
 *		4 Models are for Output Control and 
 *		4 Models for Reading the switch status which comes from External 230VAC to 3.3V.
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/drivers/gpio.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>
#include "bluetooth/mesh/gen_onoff_srv.h"
#include "model_handler.h"

#define SW0_NODE	DT_ALIAS(sw0)
#define SW1_NODE	DT_ALIAS(sw1)
#define SW2_NODE	DT_ALIAS(sw2)
#define SW3_NODE	DT_ALIAS(sw3)

static const struct gpio_dt_spec sw0_spec = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static const struct gpio_dt_spec sw1_spec = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static const struct gpio_dt_spec sw2_spec = GPIO_DT_SPEC_GET(SW2_NODE, gpios);
static const struct gpio_dt_spec sw3_spec = GPIO_DT_SPEC_GET(SW3_NODE, gpios);

static const struct gpio_dt_spec sw_spec[] = {	GPIO_DT_SPEC_GET(SW0_NODE, gpios), 
												GPIO_DT_SPEC_GET(SW1_NODE, gpios),
												GPIO_DT_SPEC_GET(SW2_NODE, gpios),
												GPIO_DT_SPEC_GET(SW3_NODE, gpios)};
/** Context for a led set and get commands and their responses. */
static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp);

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp);

/** Context for a single light server. */
static const struct bt_mesh_onoff_srv_handlers onoff_handlers = {
	.set = led_set,
	.get = led_get,
};


/** Context for a button set and get commands and their responses. */
static void button_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp);

static void button_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp);

/** Context for a single light server. */
static const struct bt_mesh_onoff_srv_handlers button_onoff_handlers = {
	.set = button_set,
	.get = button_get,
};

/*  */
/** Context for a single light switch. */
struct button_rd {
	/** Current light status of the corresponding server. */
	bool value;
	/** Generic OnOff client instance for this switch. */
	struct bt_mesh_onoff_srv srv;
};

struct led_ctx {
	struct bt_mesh_onoff_srv srv;
	struct k_work_delayable work;
	uint32_t remaining;
	bool value;
};

static struct led_ctx led_ctx[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&onoff_handlers) },
#endif
};

static struct button_rd button_rd[] = {
#if DT_NODE_EXISTS(DT_ALIAS(sw0))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&button_onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw1))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&button_onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw2))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&button_onoff_handlers) },
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw3))
	{ .srv = BT_MESH_ONOFF_SRV_INIT(&button_onoff_handlers) },
#endif
};

static void led_transition_start(struct led_ctx *led)
{
	int led_idx = led - &led_ctx[0];

	/* As long as the transition is in progress, the onoff
	 * state is "on":
	 */
	dk_set_led(led_idx, true);
	k_work_reschedule(&led->work, K_MSEC(led->remaining));
	led->remaining = 0;
}

static void led_status(struct led_ctx *led, struct bt_mesh_onoff_status *status)
{
	/* Do not include delay in the remaining time. */
	status->remaining_time = led->remaining ? led->remaining :
		k_ticks_to_ms_ceil32(k_work_delayable_remaining_get(&led->work));
	status->target_on_off = led->value;
	/* As long as the transition is in progress, the onoff state is "on": */
	status->present_on_off = led->value || status->remaining_time;
}


static void button_status(struct button_rd *button, struct bt_mesh_onoff_status *status)
{
	/* As long as the transition is in progress, the onoff state is "on": */
	status->present_on_off = button->value;
}

static void led_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);
	int led_idx = led - &led_ctx[0];

	if (set->on_off == led->value) {
		goto respond;
	}

	led->value = set->on_off;
	if (!bt_mesh_model_transition_time(set->transition)) {
		led->remaining = 0;
		dk_set_led(led_idx, set->on_off);
		goto respond;
	}

	led->remaining = set->transition->time;

	if (set->transition->delay) {
		k_work_reschedule(&led->work, K_MSEC(set->transition->delay));
	} else {
		led_transition_start(led);
	}

respond:
	if (rsp) {
		led_status(led, rsp);
	}
}

static void button_set(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    const struct bt_mesh_onoff_set *set,
		    struct bt_mesh_onoff_status *rsp)
{
	struct button_rd *button = CONTAINER_OF(srv, struct button_rd, srv);

	{
		goto respond;
	}

respond:
	if (rsp) {
		button_status(button, rsp);
	}
}

static void led_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp)
{
	struct led_ctx *led = CONTAINER_OF(srv, struct led_ctx, srv);

	led_status(led, rsp);
}

static void button_get(struct bt_mesh_onoff_srv *srv, struct bt_mesh_msg_ctx *ctx,
		    struct bt_mesh_onoff_status *rsp)
{

	/* Assign the button_rd address to the locally created button struct*/
	struct button_rd *button = CONTAINER_OF(srv, struct button_rd, srv);

	uint8_t button_idx = button - &button_rd[0];

	/* Update the button value with the latest button value */
	button -> value =  gpio_pin_get_dt(&sw_spec[button_idx]);
	
	/* update the button value to the blutooth status */
	button_status(button, rsp);
}

static void led_work(struct k_work *work)
{
	struct led_ctx *led = CONTAINER_OF(work, struct led_ctx, work.work);
	int led_idx = led - &led_ctx[0];

	if (led->remaining) {
		led_transition_start(led);
	} else {
		dk_set_led(led_idx, led->value);

		/* Publish the new value at the end of the transition */
		struct bt_mesh_onoff_status status;

		led_status(led, &status);
		bt_mesh_onoff_srv_pub(&led->srv, NULL, &status);
	}
}

static void button_handler_cb(uint32_t pressed, uint32_t changed)
{
	struct button_rd *button;

	if (!bt_mesh_is_provisioned()) {
		return;
	}

	if (IS_ENABLED(CONFIG_BT_MESH_LOW_POWER) && (pressed & changed & BIT(3))) {
		bt_mesh_proxy_identity_enable();
		return;
	}

	/* Iterate through all the buttons if there is change */
	for (int i = 0; i < ARRAY_SIZE(button_rd); ++i) 
	{
		/* Assign the button_rd address to the locally created button struct*/
		button = CONTAINER_OF(&button_rd[i].srv, struct button_rd, srv);

		/* Publish the new value at the end of the transition */
		struct bt_mesh_onoff_status status;

		/* Update the button value with the latest button value */
		button[i].value =  gpio_pin_get_dt(&sw_spec[i]);
		
		/* update the button value to the blutooth status */
		button_status(button, &status);
		
		/* Publish the status */
		bt_mesh_onoff_srv_pub(&button_rd[i].srv, NULL, &status);
	}
}

/* Set up a repeating delayed work to blink the DK's LEDs when attention is
 * requested.
 */
static struct k_work_delayable attention_blink_work;
static bool attention;

static void attention_blink(struct k_work *work)
{
	static int idx;
	const uint8_t pattern[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
		BIT(0),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
		BIT(1),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
		BIT(2),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
		BIT(3),
#endif
	};

	if (attention) {
		/* dk_set_leds(pattern[idx++ % ARRAY_SIZE(pattern)]); */
		k_work_reschedule(&attention_blink_work, K_MSEC(30));
	} else {
		/* dk_set_leds(DK_NO_LEDS_MSK); */
	}
}

static void attention_on(struct bt_mesh_model *mod)
{
	attention = true;
	k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

static void attention_off(struct bt_mesh_model *mod)
{
	/* Will stop rescheduling blink timer */
	attention = false;
}

static const struct bt_mesh_health_srv_cb health_srv_cb = {
	.attn_on = attention_on,
	.attn_off = attention_off,
};

static struct bt_mesh_health_srv health_srv = {
	.cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_elem elements[] = {
#if DT_NODE_EXISTS(DT_ALIAS(led0))
	BT_MESH_ELEM(
		1, BT_MESH_MODEL_LIST(
			BT_MESH_MODEL_CFG_SRV,
			BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
			BT_MESH_MODEL_ONOFF_SRV(&led_ctx[0].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led1))
	BT_MESH_ELEM(
		2, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[1].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led2))
	BT_MESH_ELEM(
		3, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[2].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(led3))
	BT_MESH_ELEM(
		4, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&led_ctx[3].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw0))
	BT_MESH_ELEM(
		4, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&button_rd[0].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw1))
	BT_MESH_ELEM(
		4, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&button_rd[1].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw2))
	BT_MESH_ELEM(
		4, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&button_rd[2].srv)),
		BT_MESH_MODEL_NONE),
#endif
#if DT_NODE_EXISTS(DT_ALIAS(sw3))
	BT_MESH_ELEM(
		4, BT_MESH_MODEL_LIST(BT_MESH_MODEL_ONOFF_SRV(&button_rd[3].srv)),
		BT_MESH_MODEL_NONE),
#endif
};

static const struct bt_mesh_comp comp = {
	.cid = CONFIG_BT_COMPANY_ID,
	.elem = elements,
	.elem_count = ARRAY_SIZE(elements),
};

const struct bt_mesh_comp *model_handler_init(void)
{
	gpio_pin_configure_dt(&sw0_spec, GPIO_INPUT);
	gpio_pin_configure_dt(&sw1_spec, GPIO_INPUT);
	gpio_pin_configure_dt(&sw2_spec, GPIO_INPUT);
	gpio_pin_configure_dt(&sw3_spec, GPIO_INPUT);

	static struct button_handler button_handler = {
		.cb = button_handler_cb,
	};
	dk_button_handler_add(&button_handler);

	k_work_init_delayable(&attention_blink_work, attention_blink);

	for (int i = 0; i < ARRAY_SIZE(led_ctx); ++i) {
		k_work_init_delayable(&led_ctx[i].work, led_work);
	}

	return &comp;
}
