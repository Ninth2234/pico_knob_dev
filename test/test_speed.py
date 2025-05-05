import time
import micropython

nTimes = 1000

def do_something():
    x = 5
    for i in range(nTimes):
        x = x+10



def do_something2():
    x=5
    
    
    def cal(x):
        return x+10
    
    for i in range(nTimes):
        x = cal(x)

class Calculator:
    def cal(self,x):
        return x+10

def do_something3():

    x=5
    
    calculator = Calculator()
    
    for i in range(nTimes):
        x = calculator.cal(x)



def do_something4():

    x=5
    
    calculator = Calculator()
    cache_method = calculator.cal
    for i in range(nTimes):
        x = cache_method(x)

class Calculator2:
    
    @staticmethod
    def cal(x:int)->int:
        return x+10
    
def do_something5():

    x=5
    
    calculator = Calculator2()
    cache_method = calculator.cal
    for i in range(nTimes):
        x = cache_method(x)




testCases = [
    ("do_something", do_something),
    ("do_something2", do_something2),
    ("do_something3", do_something3),
    ("do_something4", do_something4),
    ("do_something5", do_something5),
]

for name, func in testCases:
    start = time.ticks_us()
    func()
    end = time.ticks_us()
    elapsed = time.ticks_diff(end, start)  # safe for wraparound
    print("Elapsed time for {}: {} us".format(name, elapsed))