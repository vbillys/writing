#! /usr/bin/env bash
pandoc --from=markdown --template=myreport.latex -s -S sanwuloc.md -o sanwuloc.tex
#/usr/local/texlive/2015/bin/x86_64-linux/xelatex sanwuloc
PATH=/usr/local/texlive/2015/bin/x86_64-linux:$PATH /usr/local/texlive/2015/bin/x86_64-linux/latexmk -xelatex sanwuloc
