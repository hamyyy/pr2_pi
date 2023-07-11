#!/usr/bin/env python3

import os
from flask import Flask, send_from_directory
import webbrowser

app = Flask(__name__)


@app.route("/")
def base():
    return send_from_directory('../dist', 'index.html')


@app.route("/<path:path>")
def home(path):
    return send_from_directory('../dist', path)


def main():
    os.system("midori -e Fullscreen http://127.0.0.1:5000/ &")
    #webbrowser.open_new('http://127.0.0.1:5000/')
    app.run(host="127.0.0.1", port=5000)
    os.system("pkill midori")


if __name__ == '__main__':
    main()
