#!/usr/bin/env python3

import os
from flask import Flask, send_from_directory
import random

app = Flask(__name__)


@app.route("/")
def base():
    return send_from_directory('../dist', 'index.html')


@app.route("/<path:path>")
def home(path):
    return send_from_directory('../dist', path)

@app.route("/charge")
def charge():
    return random.randint(0, 100)


def main():
    os.system("midori -e Fullscreen http://localhost:5750/ &")
    app.run(host="localhost", port=5750)
    os.system("pkill midori")


if __name__ == '__main__':
    main()
