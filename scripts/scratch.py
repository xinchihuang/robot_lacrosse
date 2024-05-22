import json
class Command:
    def __init__(self,state="idle",vx=0,vy=0,vw=0,r1=0,r2=0,r3=0):

        self.state = state
        self.vx = vx
        self.vy = vy
        self.vw = vw
        self.r1 = r1
        self.r2 = r2
        self.r3 = r3
    def encode(self):

        return json.dumps(self.__dict__)
    def set_attribute(self, **kwargs):
        self.state = kwargs.get('state')
        self.vx = float(kwargs.get('vx'))
        self.vy = float(kwargs.get('vy'))
        self.vw = float(kwargs.get('vw'))
        self.r1 = float(kwargs.get('r1'))
        self.r2 = float(kwargs.get('r2'))
        self.r3 = float(kwargs.get('r3'))
    def decode(self,command_string):
        attribute_dict = json.loads(command_string)
        self.set_attribute(**attribute_dict)


command=Command()
commandstr=command.encode()
print(type(commandstr))
newcommand=Command()
newcommand.decode(commandstr)
print(newcommand.vx)