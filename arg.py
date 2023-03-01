#!/usr/bin/python3 -*- coding: utf-8 -*-

import argparse

def parse_arg(): # parameter not used yet parser = 
	print("parsing arguments")

	parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)

	parser.add_argument("-k", '--keep_alive', help='start keep alive. default FALSE', required=False, action="store_true") 

	parsed_args=parser.parse_args()
	parsed_args = vars(parsed_args) # convert object to dict. get 

	return(parsed_args) # dict
