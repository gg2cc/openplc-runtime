#!/usr/bin/env python3
"""Stop PLC execution through the local OpenPLC Runtime REST API."""

import json
import ssl
import urllib.request

API_URL = "https://localhost:8443/api"
CREDENTIALS_FILE = "/opt/openplc-runtime/openplc_username_password.txt"

with open(CREDENTIALS_FILE, encoding="utf-8") as file:
    credentials = file.read().splitlines()

if len(credentials) < 2:
    raise SystemExit("Credentials file must contain username on line 1 and password on line 2.")

username, password = credentials[0].strip(), credentials[1].strip()

if not username or not password:
    raise SystemExit("Username or password in credentials file is empty.")

# OpenPLC generates a self-signed certificate by default.
ssl_context = ssl._create_unverified_context()
login_request = urllib.request.Request(
    API_URL + "/login",
    data=json.dumps({"username": username, "password": password}).encode("utf-8"),
    headers={"Content-Type": "application/json"},
    method="POST",
)

with urllib.request.urlopen(login_request, context=ssl_context, timeout=5) as response:
    token = json.loads(response.read().decode("utf-8"))["access_token"]

stop_request = urllib.request.Request(
    API_URL + "/stop-plc",
    headers={"Authorization": "Bearer " + token},
    method="GET",
)

with urllib.request.urlopen(stop_request, context=ssl_context, timeout=5) as response:
    result = json.loads(response.read().decode("utf-8"))

status = result.get("status", "")
print(status.strip() or result)
if status.strip() != "STOP:OK":
    raise SystemExit(1)
