class MyClass:
    """A simple example class"""
    "Test push"
    i = 12345

    def __init__(self):
        self.data = []

    def f(self):
        return 'hello world'

test = MyClass()
print(test.f())