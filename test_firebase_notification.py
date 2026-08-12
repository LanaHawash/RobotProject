from pathlib import Path

import firebase_admin
from firebase_admin import credentials, messaging


SERVICE_ACCOUNT_PATH = (
    Path.home()
    / "RobotProject"
    / "config"
    / "firebase-service-account.json"
)

DEVICE_TOKEN = "fV05MFkRTNOhf0hvodcrSJ:APA91bFGGdlKzGEgYT-Cdw8A65EXS6Y3BD_vuFCqkC9SRJYUrmHyzr8j-MC4sFXb5hYw-t3Tkr2NHmhQoy8dHU1E6hXokSZnsqT-9odPjz31sAc-XRFJquk"

cred = credentials.Certificate(str(SERVICE_ACCOUNT_PATH))
firebase_admin.initialize_app(cred)

message = messaging.Message(
    notification=messaging.Notification(
        title="Baby Cry Detected",
        body="RoboCare detected possible crying.",
    ),
    data={
        "type": "BABY_CRY_DETECTED",
        "mode": "MANUAL",
    },
    token=DEVICE_TOKEN,
)

response = messaging.send(message)

print("Notification sent successfully:")
print(response)