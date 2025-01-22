str = '111'
def test():
    print(str[5])
try:
    test()
except (ValueError, IndexError):
    print('ValueError')

