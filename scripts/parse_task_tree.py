#!/usr/bin/env python

import sys
import pdb
import pickle
'''
Description: A program to parse the Task Expression Language
Output => A preorder tree traversal and a YAML file
'''

Symbol = str
List = list
Number = (int, float)

def tokenize(chars):
  '''
  Teokenize the input stream of characters based on the language
  '''
  return chars.replace('(', ' ( ').replace(')', ' ) ').split()

def read_from_tokens(tokens):
  if len(tokens) == 1:
    raise SyntaxError('Unexpected EOF while reading')
  token = tokens.pop(0)
  if '(' == token:
    L = []
    while tokens[0] != ')':
      L.append(read_from_tokens(tokens))
    tokens.pop(0)
    return L
  elif ')' == token:
    raise SyntaxError('Unexpected [)] symbol')
  else:
    return atom(token)

def atom(token):
  try: return int(token)
  except ValueError:
    try: return float(token)
    except ValueError:
      return Symbol(token)

def parse(program):
  '''
  Parse the input character stream and output syntax tree
  '''
  return read_from_tokens(tokenize(program))
class Procedure(object):
  "A user-defined Scheme procedure."
  def __init__(self, parms, body, env):
    self.parms, self.body, self.env = parms, body, env
  def __call__(self, *args): 
    return eval(self.body, Env(self.parms, args, self.env))

class Env(dict):
  "An environment: a dict of {'var':val} pairs, with an outer Env."
  def __init__(self, parms=(), args=(), outer=None):
    self.update(zip(parms, args))
    self.outer = outer if outer else {}
  def find(self, var):
    "Find the innermost Env where var appears."
    return self if (var in self) else None

def repl(prompt='lis.py> '):
  "A prompt-read-eval-print loop."
  while True:
    input_ = raw_input(prompt)
    if input_:
      val = eval(parse(input_))
      if val is not None: 
        print(schemestr(val))

def schemestr(exp):
  "Convert a Python object back into a Scheme-readable string."
  if  isinstance(exp, list):
    return '(' + ' '.join(map(schemestr, exp)) + ')' 
  else:
    return str(exp)

def CreateThenObject(robot_id, node_id, parent):
  return TaskObject('THEN', 0, robot_id, node_id, parent)

def CreatePlaceObject(robot_id, node_id, parent):
  return PlaceObject('PLACE', 3, robot_id, node_id, parent)

def CreateAndObject(robot_id, node_id, parent):
  return TaskObject('AND', 2, robot_id, node_id, parent)

def CreateOrObject(robot_id, node_id, parent):
  return TaskObject('OR', 1, robot_id, node_id, parent)


class TaskObject(object):
  def __init__(self, name='', node_type=0, robot_id=0, node_id=0, parent=''):
    self.node_type = node_type
    self.robot_id = robot_id
    self.node_id = node_id
    self.name = '%s_%d_%d_%03d' % (name, node_type, robot_id, node_id)
    self.children = ['NONE']
    self.parent = parent
    self.peers = ['NONE']

  def __call__(self, *args):
    self.children = [child.name for child in args]
    print(self)
    return self

  def __repr__(self):
    string = (
      '%(name)s:\n'
      '  mask:\n'
      '    type: %(node_type)d\n'
      '    robot: %(robot_id)d\n'
      '    node: %(node_id)d\n'
      '  parent: %(parent)s\n'
      '  children: %(children)s\n'
      '  peers: %(peers)s'
      % {
        'name': self.name,
        'node_type': self.node_type,
        'robot_id': self.robot_id,
        'node_id': self.node_id,
        'parent': self.parent,
        'children': self.children,
        'peers': self.peers
      }
    )
    return string

class PlaceObject(TaskObject):
  def __init__(self, name='', node_type=0, robot_id=0, node_id=0, parent=''):
    super(PlaceObject, self).__init__(name, node_type, robot_id, node_id, parent)
    self.place_object = ''

  def __call__(self, item):
    self.place_object = item
    print(self)
    return self

  def __repr__(self):
    parent_str = super(PlaceObject, self).__repr__()
    string = '%s\n  object: %s' % (parent_str, self.place_object)
    return string

def standard_env():
  import math, operator as op
  env = Env()
  env.update(vars(math)) # sin, cos, sqrt, pi, ...
  env.update({
    '+':op.add, '-':op.sub, '*':op.mul, '/':op.div, 
    '>':op.gt, '<':op.lt, '>=':op.ge, '<=':op.le, '=':op.eq, 
    'abs':     abs,
    'append':  op.add,  
    'apply':   apply,
    'begin':   lambda *x: x[-1],
    'car':     lambda x: x[0],
    'cdr':     lambda x: x[1:], 
    'cons':    lambda x,y: [x] + y,
    'eq?':     op.is_, 
    'equal?':  op.eq, 
    'length':  len, 
    'list':    lambda *x: list(x), 
    'list?':   lambda x: isinstance(x,list), 
    'map':     map,
    'max':     max,
    'min':     min,
    'not':     op.not_,
    'null?':   lambda x: x == [], 
    'number?': lambda x: isinstance(x, Number),   
    'procedure?': callable,
    'round':   round,
    'symbol?': lambda x: isinstance(x, Symbol),
    'THEN':    CreateThenObject,
    'PLACE':   CreatePlaceObject,
    'AND':     CreateAndObject,
    'OR':      CreateOrObject,
  })
  return env
global_env = standard_env()

# def eval(x, env=global_env):
#   "Evaluate an expression in an environment."
#   if isinstance(x, Symbol):      # variable reference
#     return env[x]
#   elif not isinstance(x, List):  # constant literal
#     return x                
#   elif x[0] == 'quote':          # (quote exp)
#     (_, exp) = x
#     return exp
#   elif x[0] == 'if':             # (if test conseq alt)
#     (_, test, conseq, alt) = x
#     exp = (conseq if eval(test, env) else alt)
#     return eval(exp, env)
#   elif x[0] == 'define':         # (define var exp)
#     (_, var, exp) = x
#     env[var] = eval(exp, env)
#   else:                          # (proc arg...)
#     proc = eval(x[0], env)
#     args = [eval(arg, env) for arg in x[1:]]
#     return proc(*args)


def DepthFirstOrder(tree, i=0):
  if isinstance(tree, list):
    output = []
    for elem in tree:
      output += DepthFirstOrder(elem)
    return output
  else:
    return [tree]

def eval(x, env=global_env, func_index=[0], parent='ROOT'):
  "Evaluate an expression in an environment."
  if isinstance(x, Symbol):      # variable reference
    if env.find(x):
      value = env.find(x)[x](0, func_index[0], parent)
      func_index[0] += 1
      return value
    return x
  elif not isinstance(x, List):  # constant literal
    return x                
  elif x[0] == 'quote':          # (quote exp)
    (_, exp) = x
    return exp
  elif x[0] == 'if':             # (if test conseq alt)
    (_, test, conseq, alt) = x
    exp = (conseq if eval(test, env) else alt)
    return eval(exp, env)
  elif x[0] == 'define':         # (define var exp)
    (_, var, exp) = x
    env[var] = eval(exp, env)
  elif x[0] == 'set!':           # (set! var exp)
    (_, var, exp) = x
    env.find(var)[var] = eval(exp, env)
  elif x[0] == 'lambda':         # (lambda (var...) body)
    (_, parms, body) = x
    return Procedure(parms, body, env)
  else:                          # (proc arg...)
    proc = eval(x[0], env, parent=parent)
    args = [eval(arg, env, parent=proc.name) for arg in x[1:]]
    return proc(*args)

if __name__ == '__main__':
  # eval(parse("(define x 10)"))
  # print eval(parse("(+  4 (+ 4 (+ 4 x)))"))
  # output preorder tree traversal

  # Perform Depth first search
  string = (
    "(THEN "
      "(PLACE placemat) "
      "(AND "
        "(OR "
          "(PLACE soda) "
          "(PLACE wineglass) "
          "(PLACE cup)) "
        "(PLACE  spoon) "
        "(PLACE knife) "
        "(THEN "
          "(PLACE plate) "
          "(PLACE bowl)) "
        "(PLACE fork)))"
  )
  print(string)
  # string = '(THEN (PLACE fork) (AND (PLACE spoon) (PLACE knife)))'
  parse_str = parse(string)
  x = eval(parse_str)
  # print(parse_str)
  # order = dict()
  # for i, node in enumerate(DepthFirstOrder(parse_str)):
  #   order[node] = i
  # with open('table_setting_demo_node_graph_order.txt', 'w') as file:
  #   file.write(pickle.dumps(order))