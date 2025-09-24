"""
@brief: test relay bank resolving 
@author: Edwin 
"""

def get_bank_number(n):
    if n > 32:
        return -1 # number of relays is 32 
    if(n > 8): #bank 0
        return n % 8;
    elif n > 15:
        return n % 15
    elif n > 23:
        return n% 23
    else:
        return n;

x = get_bank_number(10)
print(x)


