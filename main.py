from robot_project.config import (
    WEB_HOST,
    WEB_PORT,
)
from robot_project.web.app import (
    app,
    start_system,
)


def main():
    start_system()

    app.run(
        host=WEB_HOST,
        port=WEB_PORT,
        debug=False,
        use_reloader=False,
    )


if __name__ == "__main__":
    main()