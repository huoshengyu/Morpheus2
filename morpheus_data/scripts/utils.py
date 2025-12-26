#!/usr/bin/env python

# Keyboard input imports
from pynput import keyboard
from readchar import readkey, readchar, key

def get_key():
    k = readchar()
    return k

def is_enter(k):
    return k == key.ENTER or k == key.ENTER_2

def is_esc(k):
    return k == key.ESC or k == key.ESC_2

def is_del(k):
    return k == key.DELETE

def is_backspace(k):
    return k == key.BACKSPACE

def is_keyboard_interrupt(k):
    return k == key.CTRL_C

def sanitize_filename(input):
    valid_punctuation = "._ "
    input = input.strip()
    input = input.replace(' ', '_')
    filename = "".join(c for c in input if c.isalnum() or c in valid_punctuation)
    return filename