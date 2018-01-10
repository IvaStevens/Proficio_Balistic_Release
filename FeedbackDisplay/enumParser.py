# cpp_enum_parser.py
#
# Posted by Mark Tolonen on comp.lang.python in August, 2009,
# Used with permission.
# 
# Parser that scans through C or C++ code for enum definitions, and
# generates corresponding Python constant definitions.

from pyparsing import *

# Pull in enum containing string
enumFile = ""
with open('data.txt', 'r') as myfile:
    enumStr = enumfile.read().replace('\n', '')

class Map(dict):
    """
    Example:
    m = Map({'first_name': 'Eduardo'}, last_name='Pool', age=24, sports=['Soccer'])
    """
    def __init__(self, *args, **kwargs):
        super(Map, self).__init__(*args, **kwargs)
        for arg in args:
            if isinstance(arg, dict):
                for k, v in arg.iteritems():
                    self[k] = v

        if kwargs:
            for k, v in kwargs.iteritems():
                self[k] = v

    def __getattr__(self, attr):
        return self.get(attr)

    def __setattr__(self, key, value):
        self.__setitem__(key, value)

    def __setitem__(self, key, value):
        super(Map, self).__setitem__(key, value)
        self.__dict__.update({key: value})

    def __delattr__(self, item):
        self.__delitem__(item)

    def __delitem__(self, key):
        super(Map, self).__delitem__(key)
        del self.__dict__[key]


# Syntax we don't want to see in the final parse tree
LBRACE,RBRACE,EQ,COMMA = map(Suppress,"{}=,")
_enum = Suppress('enum')
identifier = Word(alphas,alphanums+'.')
integer = Word(nums)
enumValue = Group(identifier('name') + Optional(EQ + integer('value')))
enumList = Group(enumValue + ZeroOrMore(COMMA + enumValue))
enum = _enum + identifier('enum') + LBRACE + enumList('names') + RBRACE

# Create a map to house enums
EnumMap = {}

# Find instances of enums ignoring other syntax
for item,start,stop in enum.scanString(enumStr):
    id = 0
    temp = {}
    for entry in item.names:
        if entry.value != '':
            id = int(entry.value)
        temp[entry.name] = id
        print('%s.%s = %d' % (item.enum,entry.name,id))
        id += 1
    m = Map(temp)
    EnumMap[item.enum] = m

# Return the enums
ENUMS = Map(EnumMap)
