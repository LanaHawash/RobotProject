from flask import Blueprint, request


manual_drive_blueprint = Blueprint(
    "manual_drive",
    __name__,
)


def configure_manual_drive_routes(
    manual_drive_controller,
    arduino,
    navigator,
    deep_cleaning,
):
    """
    Connect the manual-drive API to the existing
    RobotProject hardware objects.
    """

    @manual_drive_blueprint.post("/movement")
    def manual_movement():
        if not arduino.is_connected():
            return {
                "success": False,
                "error": "Arduino is not connected.",
            }, 503

        data = request.get_json(silent=True)

        if not isinstance(data, dict):
            return {
                "success": False,
                "error": "JSON body is required.",
            }, 400

        command = str(
            data.get("command", "")
        ).strip().upper()

        if (
            navigator.is_running()
            or deep_cleaning.is_running()
        ):
            return {
                "success": False,
                "error": (
                    "Manual movement is unavailable "
                    "while autonomous navigation "
                    "is running."
                ),
            }, 409

        try:
            manual_drive_controller.move(command)

        except ValueError as error:
            return {
                "success": False,
                "error": str(error),
            }, 400

        except Exception as error:
            return {
                "success": False,
                "error": str(error),
            }, 500

        return {
            "success": True,
            "command": command,
        }

    return manual_drive_blueprint