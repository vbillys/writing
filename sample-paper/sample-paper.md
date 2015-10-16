---  
title: A sample paper  
author: Donald Duck
date: October 1, 2014  
bibliography: [sample-library.bib, isam.bib]
csl: ieee.csl # chicago.csl
reference-docx: path to your style template for MS Word
abstract: Lorem ipsum dolor sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enimad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. Duis aute irure dolor in reprehenderit in voluptate velit esse cillum dolore eu fugiat nulla pariatur. Excepteur sint occaecat cupidatat non proident, sunt in culpa qui officia deserunt mollit anim id est laborum.
---  

# Section 1  

## Subsection 1.1
Lorem *ipsum* **dolor** sit amet, consectetur adipisicing elit, sed do eiusmod tempor incididunt ut labore et dolore magna aliqua. Ut enim ad minim veniam, quis nostrud exercitation ullamco laboris nisi ut aliquip ex ea commodo consequat. See Table \ref{table_example}.

\begin{table}[!t]
\renewcommand{\arraystretch}{1.3}
\caption{A Simple Example Table, IEEE standard format}
\label{table_example}
\centering
\begin{tabular}{c||c}
\hline
\bfseries First & \bfseries Next\\
\hline\hline
1.0 & 2.0\\
\hline
\end{tabular}
\end{table}

:Table Example from Markdown syntax \label{table_markdown}

fruit| price
-----|-----:
apple|2.05
pear|1.37
orange yeah referencing from inside table to \ref{table_markdown}!|3.09

## Subsection 1.2
Sed ut perspiciatis unde omnis iste natus error sit voluptatem accusantium doloremque laudantium, totam rem aperiam, eaque  ipsa quae ab illo inventore veritatis et quasi architecto beatae vitae dicta sunt explicabo.

- one item
- two items
- three items
- four items

Sed ut perspiciatis unde omnis iste natus error sit voluptatem accusantium doloremque laudantium, totam rem aperiam, eaque  ipsa quae ab illo inventore veritatis et quasi architecto beatae vitae dicta sunt explicabo.

You can enter LaTeX equations as inline math: such as $({e}^{i\pi }+1=0) \label{eqn_md1}$ or refering \ref{eqn_md1}, \ref{eqn_md2} and \ref{eqn_example}:

$\mathbf{V}_1 \times \mathbf{V}_2 =  \begin{vmatrix}
\mathbf{i} & \mathbf{j} & \mathbf{k} \\
\frac{\partial X}{\partial u} &  \frac{\partial Y}{\partial u} & 0 \\
\frac{\partial X}{\partial v} &  \frac{\partial Y}{\partial v} & 0
\end{vmatrix} \label{eqn_md2}$

\begin{equation}
\label{eqn_example}
x = \sum\limits_{i=0}^{z} 2^{i}Q
\end{equation}

# Section 2

## Subsection 2.1
Sed ut perspiciatis unde omnis iste natus error sit voluptatem accusantium doloremque laudantium, totam rem aperiam, eaque  ipsa quae ab illo inventore veritatis et quasi architecto beatae vitae dicta sunt explicabo.

Sed ut perspiciatis unde omnis iste natus error sit voluptatem accusantium doloremque laudantium, totam rem aperiam, eaque  ipsa quae ab illo inventore veritatis et quasi architecto beatae vitae dicta sunt explicabo [link](http://www.http://daringfireball.net). You enter citations using @ and bibtex citation key, such as @donaldduck2014 if you want only the year in brackets, or [@donaldduck2013;@donaldduck2014;@Kaess08tro] for a normal ciation in author-date form within brackets. 

This is the code for an image. Much simpler than in LaTeX. But you can also use LaTeX code here. Just keep in mind that LaTeX code is only rendered when converted to PDF. Conversion to Word or html won't pick it up. And see that figure referenced see \ref{fig:mylabel}.

Term 1

:   Definition 1

Term 2 with *inline markup*

:   Definition 2

        { some code, part of Definition 2 }

    Third paragraph of definition 2.

(@)  My first example will be numbered (1).
(@)  My second example will be numbered (2).

Explanation of examples.

(@)  My third example will be numbered (3).

<http://google.com>

<sam@green.eggs.ham>

[my label 1]: /foo/bar.html  "My title, optional"
[my label 2]: /foo
[my label 3]: http://fsf.org (The free software foundation)
[my label 4]: /bar#special  'A title in single quotes'
[my label 5]: <http://foo.bar.baz>

See [my website][], or [my website].

[my website]: http://foo.bar.baz

#### heading what one [header identifiers]

#### heading what one [the section on header identifiers]

## My header ##    {#foo}

See the [My Header](#foo) or [this](#header-identifiers), [my label 2] or [my label 1] and [my label 3], also [my label 4], and lastly [my label 5]

Here is a footnote reference,[^1] and another.[^longnote]

[^1]: Here is the footnote.

[^longnote]: Here's one with multiple blocks.

    Subsequent paragraphs are indented to show that they
belong to the previous footnote.

        { some.code }

    The whole paragraph can be indented, or just the first
    line.  In this way, multi-paragraph footnotes work like
    multi-paragraph list items.

This paragraph won't be part of the note, because it
isn't indented.

![image       \label{fig:mylabel}  caption](sample-image.jpg "beautiful cat")

# References

The reference list is added here automatically by Pandoc:

\setlength{\parindent}{-0.2in} \setlength{\leftskip}{0.2in} \setlength{\parskip}{8pt} \vspace*{-0.2in} \noindent

