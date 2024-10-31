import configparser
import os

def before_build(source, target, env):
    # 設定ファイルを読み込む
    config = configparser.ConfigParser()
    config.read('secrets.ini')

    # WiFi情報を取得
    ssid = config.get('env:secrets', 'SSID')
    password = config.get('env:secrets', 'PASSWORD')

    # ファイルを読み込んでプレースホルダーを置換
    with open('src/main.cpp', 'r') as file:
        src = file.read()

    src = src.replace('"SSID"', '"' + ssid + '"')
    src = src.replace('"PASSWORD"', '"' + password + '"')

    # 変更内容をファイルに書き込む
    with open('src/main.cpp', 'w') as file:
        file.write(src)

Import("env")
env.AddPreAction("buildprog", before_build)
