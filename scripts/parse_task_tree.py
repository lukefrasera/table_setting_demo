#!/usr/bin/env python

import sys
import pdb
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

ENV = dict
def standard_env():
  import math, operator as op
  env = ENV()
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
  })
  return env
global_env = standard_env()

def eval(x, env=global_env):
  "Evaluate an expression in an environment."
  if isinstance(x, Symbol):      # variable reference
    return env[x]
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
  else:                          # (proc arg...)
    proc = eval(x[0], env)
    args = [eval(arg, env) for arg in x[1:]]
    return proc(*args)

if __name__ == '__main__':
  print eval(parse("(+  4 (+ 4 (+ 4 4)))"))
  # output preorder tree traversal