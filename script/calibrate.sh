#!/bin/bash
stty -F $1 -hupcl
echo -e "C $2\n" > $1