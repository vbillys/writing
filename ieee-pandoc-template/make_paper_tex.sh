#! /bin/bash
pandoc \
	--standalone \
	--natbib \
	--smart \
	--listings \
	--template=./template_conf_multian.latex \
	-H custom.latex \
	--from=markdown \
	metadata.yaml ./sections/main.md ./sections/other.md -o paper.tex
#docker run --rm -it -v $(pwd):/var/texlive harshjv/texlive-2015 xelatex paper.tex
