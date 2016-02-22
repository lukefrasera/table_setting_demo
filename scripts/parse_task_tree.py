#!/usr/bin/env python

import sys
'''
Description: A program to parse the Task Expression Language
Output => A preorder tree traversal and a YAML file
'''

Symbol = str
List = list
Number = (int, float)

def tokenize(chars):
  '''
  Teokenize the input stream of characters based on the language syntax
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

if __name__ == '__main__':
  print parse('(THEN a b (OR c d (AND e f)))')
  # output preorder tree traversal