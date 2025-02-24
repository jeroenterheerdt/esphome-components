import esphome.config_validation as cv
import esphome.codegen as cg
from esphome.components import display, uart
from esphome.const import CONF_HEIGHT, CONF_ID, CONF_LAMBDA
from esphome import automation

DEPENDENCIES = ["uart"]

thermal_printer_ns = cg.esphome_ns.namespace("thermal_printer")

ThermalPrinterDisplay = thermal_printer_ns.class_(
    "ThermalPrinterDisplay", display.DisplayBuffer, uart.UARTDevice
)

ThermalPrinterPrintTextAction = thermal_printer_ns.class_(
    "ThermalPrinterPrintTextAction", automation.Action
)

CONF_FONT_SIZE = "font_size"
CONF_TEXT = "text"

CONFIG_SCHEMA = (
    display.FULL_DISPLAY_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(ThermalPrinterDisplay),
            cv.Required(CONF_HEIGHT): cv.uint16_t,
        }
    )
    .extend(
        cv.polling_component_schema("never")
    )  # This component should always be manually updated with actions
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await display.register_display(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_height(config[CONF_HEIGHT]))

    if lambda_config := config.get(CONF_LAMBDA):
        lambda_ = await cg.process_lambda(
            lambda_config, [(display.DisplayRef, "it")], return_type=cg.void
        )
        cg.add(var.set_writer(lambda_))


@automation.register_action(
    "thermal_printer.print_text",
    ThermalPrinterPrintTextAction,
    cv.maybe_simple_value(
        cv.Schema(
            {
                cv.GenerateID(): cv.use_id(ThermalPrinterDisplay),
                cv.Required(CONF_TEXT): cv.templatable(cv.string),
                cv.Optional(CONF_FONT_SIZE, default=1): cv.templatable(
                    cv.int_range(min=0, max=7)
                ),
            }
        ),
        key=CONF_TEXT,
    ),
)
async def thermal_printer_print_text_action_to_code(
    config, action_id, template_arg, args
):
    var = cg.new_Pvariable(action_id, template_arg)
    await cg.register_parented(var, config[CONF_ID])
    templ = await cg.templatable(config[CONF_TEXT], args, cg.std_string)
    cg.add(var.set_text(templ))
    templ = await cg.templatable(config[CONF_FONT_SIZE], args, cg.uint8)
    cg.add(var.set_font_size(templ))
    return var
