import sys
sys.path.append('./static/')

import json
import numpy as np

from flask import Flask, request, render_template, redirect
from axxb import axxb, tupilize

app = Flask(__name__)

@app.route('/', methods=['GET', 'POST'])
def index():
    return redirect("https://github.com/bstadt/pmCalibrate")

@app.route('/calc', methods = ['POST'])
def calc():
    if request.method == 'POST':
        jsonObj = json.loads(request.data)
        Rx, t = axxb(np.array(jsonObj['Qee']), np.array(jsonObj['Qmarker']))
        return json.dumps({'Rx': tupilize(Rx), 't':tupilize(t)})
    else:
        return 400
'''
if __name__ == '__main__':
    app.run(port=8000)
'''
