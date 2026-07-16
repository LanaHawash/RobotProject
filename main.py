from robot_project.config import WEB_HOST, WEB_PORT
from robot_project.web.app import app


if __name__ == "__main__":
    app.run(
        host=WEB_HOST,
        port=WEB_PORT,
        debug=False,
        use_reloader=False,
        threaded=True,
    )