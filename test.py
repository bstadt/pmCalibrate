import requests
import json

Qee =((0.504,-0.082,0.371,0.141,0.737,-0.124,0.649),
      (0.511,-0.018,0.371,0.094,0.744,-0.083,0.656),
      (0.485,-0.071,0.312,0.291,0.724,-0.302,0.547))

Qmarker = ((-0.079,0.038,0.362,0.373,0.648,-0.561,-0.355),
           (-0.046,-0.016,0.360,0.335,0.669,-0.583,-0.315),
           (-0.102,0.042,0.296,0.535,0.574,-0.413,-0.462))

response = requests.post('http://park-martin-calibrate.herokuapp.com/calc',
                          data=json.dumps({'Qee':Qee, 'Qmarker':Qmarker}))

responseObj = json.loads(response.text)
print responseObj['Rx']
print responseObj['t']
