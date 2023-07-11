#!/usr/bin/env python3

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
    webbrowser.open_new('http://127.0.0.1:5000/')
    app.run(host="127.0.0.1", port=5000)


if __name__ == '__main__':
    main()