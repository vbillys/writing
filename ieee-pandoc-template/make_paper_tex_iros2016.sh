#!/usr/bin/zsh
pandoc \
	--standalone \
	--natbib \
	--smart \
	--listings \
	--template=./template_conf_multian_iros2016.latex \
	-H custom.latex \
	--from=markdown \
	metadata_iros2016.yaml ../IROSPaper.md ./sections/other.md -o paper_iros2016.tex
