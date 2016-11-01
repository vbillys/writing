#! /bin/bash
docker run --rm -it -v $(pwd):/var/texlive harshjv/texlive-2015 xelatex paper.tex
