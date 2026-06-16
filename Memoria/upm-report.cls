\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{upm-report}  %[2023-05-31 v1.0 UPM Report Template]

\DeclareOption*{\PassOptionsToClass{\CurrentOption}{book}}
\ProcessOptions\relax
\LoadClass[12pt,a4paper,oneside]{book}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Nos aseguramos de que usamos xetex como motor de compilación
%
\RequirePackage{ifxetex}
\RequireXeTeX

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Paquetes globales
%
\RequirePackage[titletoc]{appendix}  % Que aparezca "Apéndice"
\RequirePackage{awesomebox}  % Bloques de texto más molones
\RequirePackage{changepage}  % Para poder usar los adjustwidth
\RequirePackage{csquotes}  % Facilitar el trabajo con citas
\RequirePackage{eso-pic}  % Colores y fondos
\RequirePackage{graphicx}  % Mejoras sobre el paquete graphics
\RequirePackage{hologo}  % Para poner logos relacionados con LaTeX
\RequirePackage{etoolbox}  % Para poder usar condicionales en la clase
\RequirePackage{pdflscape}  % Posibilidad de páginas en apaisado
\RequirePackage{xcolor}  % Mejoras sobre el paquete color
\RequirePackage{algorithm}
\RequirePackage{algpseudocode}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Valores constantes
%
\def\universityname{Universidad Politécnica de Madrid}
\def\templaterepository{https://github.com/blazaid/UPM-Report-Template}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Opciones de la clase
%
\RequirePackage{kvoptions} % Para usar argumentos al importar la clase
\SetupKeyvalOptions{family=@upm, prefix=@upm@}

% Opciones de configuración de la clase
\DeclareStringOption{type}
\DeclareStringOption{school}
\DeclareStringOption{degree}
\ProcessKeyvalOptions*

% Variables relacionadas con el tipo de memoria (arg. `type`)

% Registramos los tipos de memoria ...
\newcommand{\RegisterType}[3]{%
    \csdef{upm@typename@#1}{#2}%
    \csdef{upm@typeabbr@#1}{#3}%
}
\RegisterType{pfg}{Proyecto Fin de Grado}{PFG}
\RegisterType{tfg}{Trabajo Fin de Grado}{TFG}
\RegisterType{pfm}{Proyecto Fin de Máster}{PFM}
\RegisterType{tfm}{Trabajo Fin de Máster}{TFM}
\RegisterType{td}{Tesis Doctoral}{PhD}

% ... y cogemos la indicada en el argumento `type`
\newcommand{\reporttype}{NO DEFINIDO}
\newcommand{\reporttypeabbr}{NO DEFINIDO}
\ifcsdef{upm@typename@\@upm@type}{%
    \renewcommand{\reporttype}{\csuse{upm@typename@\@upm@type}}%
    \renewcommand{\reporttypeabbr}{\csuse{upm@typeabbr@\@upm@type}}%
}{\PackageError{upm-report}{Type}{Parámetro 'type' no válido o faltante}}

% Variables relacionadas con la escuela (arg. `school`)

% Registramos las escuelas ...
\newcommand{\RegisterSchool}[8]{%
    \csdef{upm@schoolname@#1}{#2}%
    \csdef{upm@schoolabbr@#1}{#3}%
    \csdef{upm@schoollogo@#1}{#4}%
    \csdef{upm@schoollogoblack@#1}{#5}%
    \csdef{upm@schooladdress@#1}{#6}%
    \csdef{upm@schoolcolor@#1}{#7}%
    \csdef{upm@linkcolor@#1}{#8}%
}
\RegisterSchool{etsiaab}%
    {E.T.S. de Ingeniería Agronómica, Alimentaria y de Biosistemas}%
    {ETSIAAB}%
    {upm-report/images/etsiaab-logotipo.png}%
    {upm-report/images/etsiaab-logotipo.png}%
    {Campus Ciudad Universitaria\\Av. Puerta de Hierro, nº 2 - 4\\28040 Madrid, España}%
    {99,178,76}%
    {99,178,76}%
\RegisterSchool{etsidi}%
    {E.T.S. de Ingeniería y Diseño Industrial}%
    {ETSIDI}%
    {upm-report/images/etsidi-logotipo.png}%
    {upm-report/images/etsidi-logotipo-fondonegro.pdf}%
    {c/ Ronda de Valencia, 3\\28012, Madrid, España}%
    {223,30,64}%
    {223,30,64}
\RegisterSchool{etsisi}%
    {E.T.S. de Ingeniería de Sistemas Informáticos}%
    {ETSISI}%
    {upm-report/images/etsisi-isotipo.pdf}%
    {upm-report/images/etsisi-isotipo-white.pdf}%
    {Campus Sur UPM, Carretera de Valencia (A-3), km. 7\\28031, Madrid, España}%
    {0,114,206}%
    {0,114,206}

% ... y cogemos la indicada en el argumento `school`
\newcommand{\schoolname}{NO DEFINIDO}
\newcommand{\schoolabbr}{NO DEFINIDO}
\newcommand{\schoollogo}{NO DEFINIDO}
\newcommand{\schoollogoblackbackground}{NO DEFINIDO}
\newcommand{\schooladdress}{NO DEFINIDO}

\ifcsdef{upm@schoolname@\@upm@school}{%
    \renewcommand{\schoolname}{\csuse{upm@schoolname@\@upm@school}}%
    \renewcommand{\schoolabbr}{\csuse{upm@schoolabbr@\@upm@school}}%
    \renewcommand{\schoollogo}{\csuse{upm@schoollogo@\@upm@school}}%
    \renewcommand{\schoollogoblackbackground}{\csuse{upm@schoollogoblack@\@upm@school}}%
    \renewcommand{\schooladdress}{\csuse{upm@schooladdress@\@upm@school}}%
    \definecolor{schoolcolor}{RGB}{\csuse{upm@schoolcolor@\@upm@school}}%
    \definecolor{linkcolor}{RGB}{\csuse{upm@linkcolor@\@upm@school}}%
}{\PackageError{upm-report}{School}{Parámetro 'school' no válido o faltante}}

% Variables relacionadas con la titulación (arg. `degree`)

% Registramos los grados ...
\newcommand{\RegisterDegree}[2]{\csdef{upm@degreename@#1}{#2}}
%% ETSAAB
\RegisterDegree{20BT}{Grado en Biotecnología}
\RegisterDegree{20BI}{Grado en Ciencias Agrarias y Bioeconomía}
\RegisterDegree{20IG}{Grado en Ingeniería Agrícola}
\RegisterDegree{02IA}{Grado en Ingeniería Agroambiental}
\RegisterDegree{20IA}{Grado en Ingeniería Alimentaria}
%% ETSIDI
\RegisterDegree{56IE}{Grado en Ingeniería Eléctrica}
\RegisterDegree{56IA}{Grado en Ingeniería Electrónica Industrial y Automática}
\RegisterDegree{56IM}{Grado en Ingeniería Mecánica}
\RegisterDegree{56IQ}{Grado en Ingeniería Química}
\RegisterDegree{56DD}{Grado en Ingeniería en Diseño Industrial y Desarrollo de Producto}
%% ETSISI
\RegisterDegree{61CD}{Grado en Ciencia de Datos e Inteligencia Artificial}
\RegisterDegree{61CI}{Grado en Ingeniería de Computadores}
\RegisterDegree{61IW}{Grado en Ingeniería del Software}
\RegisterDegree{61SI}{Grado en Sistemas de Información}
\RegisterDegree{61TI}{Grado en Tecnologías para la Sociedad de la Información}
\RegisterDegree{MAADM}{Máster Universitario en Aprendizaje Automático y Datos Masivos}
\RegisterDegree{MPBD}{Máster en Proyectos Big Data}

% ... y cogemos el indicada en el argumento `degree`
\newcommand{\degreecode}{NO DEFINIDO}
\newcommand{\degreename}{NO DEFINIDO}
\ifcsdef{upm@degreename@\@upm@degree}{%
    \renewcommand{\degreecode}{\@upm@degree}%
    \renewcommand{\degreename}{\csuse{upm@degreename@\@upm@degree}}%
}{\PackageError{upm-report}{Degree}{Parámetro 'degree' no válido}}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autores, directores y demás
%
\renewcommand{\and}{y\space}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Variables
%
\makeatletter
\newcommand{\bibauthor}[1]{\gdef\@bibauthor{#1}}%
\newcommand{\@bibauthor}{\@latex@warning@no@line{No \noexpand\bibauthor given}}
\newcommand{\director}[1]{\gdef\@director{#1}}%
\newcommand{\@director}{\@latex@warning@no@line{No \noexpand\director given}}

\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Idioma
%
\RequirePackage[spanish,es-tabla,es-ucroman]{babel}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Geometría general de las páginas
%
\RequirePackage[
    top=3.5cm,
    bottom=3cm,
    left=2.5cm,
    right=2.5cm,
]{geometry}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cabeceras y pies de página
%
\RequirePackage{fancyhdr}
\setlength{\headheight}{15.8pt}
\pagestyle{fancy}

% Estilos globales
\fancyhf{}
\fancyhead[L]{\textcolor{schoolcolor}{\sffamily\nouppercase{\rightmark}}}
\fancyhead[R]{\textcolor{schoolcolor}{\sffamily\thepage}}
\renewcommand{\headrulewidth}{0.5pt}

% Estilos en páginas "plain" (e.g. primera de capítulo)
\fancypagestyle{plain}{%
    \fancyhf{}
    \renewcommand{\headrule}{}
    \renewcommand{\footrule}{}
}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estilos de capítulo, sección, ...
%
\RequirePackage{titlesec}

\titlespacing*{\chapter}{0pt}{0pt}{25pt}
\titleformat{\chapter}%
    {\fontsize{30}{30}}%
    {\sffamily\fontsize{30}{30}\color{schoolcolor}\thechapter.}%
    {0.6em}%
    {\sffamily\fontsize{30}{30}\raggedleft\hspace*{\fill}}
    [\hrule]

\titlespacing*{\section}{0pt}{1ex plus .5ex minus .2ex}{.8ex plus .3ex}
\titleformat{\section}%
    {}{\sffamily\huge\thesection.}{0.6em}{\sffamily\huge}
\titleformat{\subsection}%
    {}{\sffamily\Large\thesubsection.}{0.6em}{\sffamily\Large}
\titleformat{\subsubsection}%
    {}{\sffamily\large\thesubsubsection.}{0.6em}{\sffamily\large}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estilos de la tabla de contenidos
%
\RequirePackage{titletoc}
\setcounter{tocdepth}{1}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fuentes
%
\RequirePackage{fontspec}  % Para importar fuentes externas
\RequirePackage{microtype} % Mejoras tipográficas generales
% TODO Cambiar los tipos de letra según la guía de estilos de la escuela
%\setmainfont{TeX Gyre PagellaX}
\setmainfont{Source Serif Pro}
\setsansfont{Nimbus Sans}
\setmonofont{Source Code Pro}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Fórmulas matemáticas
%
\RequirePackage{amsmath}   % Mejoras presentación matemática
\RequirePackage{amssymb}   % Símbolos matemáticos
\RequirePackage{mathtools} % Más mejoras de presentación matemática
\RequirePackage{xfrac}
\newcommand{\norm}[1]{\lVert#1\rVert}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Configuración de figuras y tablas
%
\RequirePackage{float}      % Mejorar colocación de elementos flotantes
\RequirePackage{tabularx}   % Tablas más configurables
\newcolumntype{C}{%
    >{\centering\arraybackslash}X%
    }                       % Para tener columnas tipo X centradas
\RequirePackage{booktabs}   % Configuraciones para tablas más bonitas
\RequirePackage{caption}    % Mejora de pies de figuras
\RequirePackage{subcaption} % Pies de subfiguras


\DeclareCaptionFormat{caption-upm}{#1#2~\footnotesize #3}
\DeclareCaptionLabelFormat{caption-upm}{\footnotesize\textbf{#1 #2}}
\DeclareCaptionLabelSeparator{caption-upm}{.}

\captionsetup{
    format=caption-upm,
    labelformat=caption-upm,
    labelsep=caption-upm,
}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cálculo de la clave en la entrada de bibtex
% 
% YYY + autor sin espacios y en minúsculas; vacío si no hay autor
%
\makeatletter

\ExplSyntaxOn

% \printauthorcompact -> toma \@author y lo limpia
\NewDocumentCommand{\cleanauthor}{}{
  \tl_set:Nx \l_tmpa_tl { \@author }
  % 1) quita \and (y espacios alrededor)
  \regex_replace_all:nnN { \s*\\and\s* } { } \l_tmpa_tl
  % 2) elimina TODO lo que no sea ASCII alfanumérico (también quita espacios y signos)
  \regex_replace_all:nnN { [^A-Za-z0-9]+ } { } \l_tmpa_tl
  % 3) imprime
  \tl_use:N \l_tmpa_tl
}

\ExplSyntaxOff

\newcommand{\generatekey}{\the\year\cleanauthor}

\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Configuración general de espaciados, geometrías y otros
%
\setlength{\parskip}{1em}
\linespread{1.2}
\setlength{\parindent}{0pt}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estilo de los enumerados y listados
%
\RequirePackage{enumitem}
\setlist[itemize,enumerate]{left=0pt..1em}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Configuración para PDF e hipervínculos
%
\RequirePackage[naturalnames]{hyperref}
\makeatletter
\hypersetup{
    pdfauthor={\@author},
    pdftitle={\@title},
    %pdfsubject={},   % TODO PONER ALGUNA OPCIÓN PARA DETERMINAR EL TEMA
    %pdfkeywords={},  % TODO QUE LAS KEYWORDS SE USEN TAMBIÉN AQUÍ
    pdfcreator={\@author},
    pdfproducer={},
    breaklinks=true,
    linktoc=all,
    plainpages=false,
    unicode=true,
    colorlinks=true,
    anchorcolor={linkcolor},
    citecolor={linkcolor},
    linkcolor={linkcolor},
    urlcolor={linkcolor},
}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Estilos de los listados de código fuente
%
\RequirePackage{listings}
\renewcommand{\lstlistingname}{Listado}
\renewcommand*{\lstlistlistingname}{Índice de listados}

\makeatletter
\renewcommand\lstlistoflistings{\bgroup
  \let\contentsname\lstlistlistingname
  \def\l@lstlisting##1##2{\@dottedtocline{1}{1.5em}{2.3em}{##1}{##2}}
  \let\lst@temp\@starttoc \def\@starttoc##1{\lst@temp{lol}}%
  \tableofcontents \egroup}
\makeatother

\definecolor{sbase03}{HTML}{002B36}
\definecolor{sbase02}{HTML}{073642}
\definecolor{sbase01}{HTML}{586E75}
\definecolor{sbase00}{HTML}{657B83}
\definecolor{sbase0}{HTML}{839496}
\definecolor{sbase1}{HTML}{93A1A1}
\definecolor{sbase2}{HTML}{EEE8D5}
\definecolor{sbase3}{HTML}{FDF6E3}
\definecolor{syellow}{HTML}{B58900}
\definecolor{sorange}{HTML}{CB4B16}
\definecolor{sred}{HTML}{DC322F}
\definecolor{smagenta}{HTML}{D33682}
\definecolor{sviolet}{HTML}{6C71C4}
\definecolor{sblue}{HTML}{268BD2}
\definecolor{scyan}{HTML}{2AA198}
\definecolor{sgreen}{HTML}{859900}
\lstset{
    sensitive=true,
    aboveskip=\baselineskip,
    belowskip=\baselineskip,
    frame=lines,
    xleftmargin=\parindent,
    belowcaptionskip=1\baselineskip,
    basicstyle=\color{sbase00}\ttfamily,
    keywordstyle=\color{scyan},
    commentstyle=\color{sbase1},
    stringstyle=\color{sblue},
    numberstyle=\color{sviolet},
    identifierstyle=\color{sbase00},
    breaklines=true,
    showstringspaces=false,
    tabsize=2
}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Listados condicionales
%
\makeatletter
\newcommand\conditionalLoF{%
    \IfFileExists{\jobname.lof}{\listoffigures}{}%
}
\newcommand\conditionalLoT{%
    \IfFileExists{\jobname.lot}{\listoftables}{}%
}
\newcommand\conditionalLoL{%
    \IfFileExists{\jobname.lol}{\lstlistoflistings}{}%
}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Licencia de la memoria
%
\RequirePackage[%
    type={CC},%
    modifier={by-nc-sa},%
    version={4.0}%
]{doclicense}

\makeatletter
% Impresión de licencia modificada para que incluya más cosas. Es un
% clon de \docLicenseThis
\newcommand{\licencia}{%
    \setlength{\doclicense@hsize}{\linewidth-\doclicense@imagewidth-\doclicense@imagedistance}%
    \begin{center}
        \begin{minipage}{\doclicense@hsize}
            \ifdefvoid{\doclicense@hyphenation}{}{%
                \@nameuse{\doclicense@hyphenation} 
            }%
            \doclicenseLongText Obra derivada de \href{\templaterepository}{\templaterepository}.%
        \end{minipage}
        \hfill
        \begin{minipage}{\doclicense@imagewidth}\doclicenseImage\end{minipage}
    \end{center}
    \vspace{-1.5em}
    Todo cambio respecto a la obra original es responsabilidad exclusiva del presente autor.\\
}
\makeatother
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Referencias y bibliografía
%
\RequirePackage[backend=biber,style=ieee]{biblatex}

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Portada
%
% No hay una genérica porque cada escuela/titulación/faraón quiere la
% suya propia, así que no queda otra que ir separando por requisitos. En
% el `else` principal está la genérica. A lo mejor en un futuro se puede
% tener un formato común.
\makeatletter         
\ifdefstring{\@upm@school}{etsidi}{%
\renewcommand\maketitle{{
    \sffamily
    \thispagestyle{empty}

    \noindent%
    \includegraphics[height=6em]{upm-report/images/etsisi-logotipo-fondoblanco.png}
    \hfill%
    \includegraphics[height=6em]{upm-report/images/upm-logotipo.png}%

    \begin{center}
        {\Large \MakeUppercase{\universityname}}\\*[0em]
        {\large \MakeUppercase{\schoolname}}\\*[0.5em]
    
        \vfill
    
        {\large \MakeUppercase{\textbf{\reporttype}}}\\*[0em]
        {\large \degreename}
    
        \vfill
        
        \Large \textbf{\@title}
    
        \vfill
        
        \large \textbf{Autoría}\\*[1em]
        \@author

        \vfill
        
        \large \textbf{Dirección}\\*[1em]
        \@director
    \end{center}

    \vfill

    \begin{flushright}
        \normalsize Madrid, \@date
    \end{flushright}
}}
}{%
\renewcommand\maketitle{{
    \sffamily
    \thispagestyle{empty}
    % Barra lateral izquierda y logo fondo
    \AddToShipoutPicture*{%
        \AtPageLowerLeft{%
            \makebox[1\paperwidth][r]{%
                \includegraphics[width=0.9\paperwidth]{upm-report/images/upm-logotipo}
            }%
        }
        \put(0,0){%
            \colorbox{schoolcolor}{%
                \parbox[b][\paperheight]{2cm}{\vfill\hfill}
            }%
        }
    }
    \begin{adjustwidth}{0.5cm}{}
        {\normalsize \MakeUppercase{\universityname}}\\*[0.5em]
        {\normalsize \MakeUppercase{\textbf{\schoolname}}}\\*[0.5em]
        {\normalsize \MakeUppercase{\reporttype}}\\*[0.5em]
        {\normalsize \MakeUppercase{\textbf{\degreename}}}
        \vfill
        \begin{hyphenrules}{nohyphenation}
        \raggedright
        \begin{sloppypar}\fontsize{34}{34}\selectfont\bfseries\@title\end{sloppypar}
        \end{hyphenrules}
        \vfill
        \parbox[b]{\linewidth}{
            \normalsize
            \textbf{Desarrollado por}: \@author \\*[0.5em]
            \textbf{Dirigido por}: \@director \\*[0.5em]
            Madrid, \@date
        }
    \end{adjustwidth}
    \normalfont
    \normalsize
}}
}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Licencia
%
\makeatletter
\newcommand\makelicense{{
    \footnotesize
    \clearpage
    \thispagestyle{empty}
    ~\vfill
    \par{
        \textit{\@title}\\
        \textbf{Desarrollado por}: \@author\\
        \textbf{Dirigido por}: \@director\\
        \reporttype, \today\\
    }
    
    \par{
        \textbf{\schoolname}\\
        \schooladdress
    }
    \newline
    \hrule
    \par{
        Si deseas citar este trabajo, la entrada completa en \hologo{BibTeX} es la siguiente:

        \texttt{%
            @mastersthesis\{\generatekey,\\%
            \hspace*{2em}title   = \{\@title\},\\
            \ifdefstring{\@upm@type}{pfg}{
                \hspace*{2em}type    = \{Bachelor's Thesis\},\\
            }{}
            \hspace*{2em}author  = \{\@bibauthor\},\\
            \hspace*{2em}school  = \{\schoolname\},\\
            \hspace*{2em}year    = \{\the\year\},\\
            \hspace*{2em}month   = \{\the\month\},\\
            \}
        }
    }
    \newline
    \hrule
    \vspace*{-0.5cm}\par{\licencia}
 }}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Agradecimientos
%
\makeatletter
\newcommand{\acknowledgements}[1]{\gdef\@acknowledgements{#1}}%
\newcommand{\@acknowledgements}{}
\newcommand\makeacknowledgements{{
    \ifdefempty{\@acknowledgements}{}{%
        \clearpage
        \thispagestyle{empty}
        \chapter*{Agradecimientos}
        \@acknowledgements
    }
}}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cita a comienzo de la memoria
%
\makeatletter
\newcommand{\reportquotation}[2]{%
  \gdef\@reportquote{#1}%
  \gdef\@reportauthor{#2}%
}
\newcommand{\@reportquote}{}%
\newcommand{\@reportauthor}{}%
\newcommand\makereportquotation{%
  \ifdefempty{\@reportquote}{}{%
    \clearpage%
    \thispagestyle{empty}%
    \begin{flushright}%
    \begin{minipage}{0.8\textwidth}%
      \textit{\@reportquote}
      
      \vspace{1em}
      \hfill--- \@reportauthor
    \end{minipage}
    \end{flushright}
  }
}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Resumen y abstract
%
\makeatletter
\newcommand{\abstract}[2]{%
    \ifstrequal{#1}{spanish}{\gdef\@abstractsp{#2}}{}%
    \ifstrequal{#1}{english}{\gdef\@abstracten{#2}}{}%
}%
\newcommand{\keywords}[2]{%
    \ifstrequal{#1}{spanish}{\gdef\@keywordssp{#2}}{}%
    \ifstrequal{#1}{english}{\gdef\@keywordsen{#2}}{}%
}%

\newcommand\makeresumen{
    \clearpage
    \thispagestyle{empty}
    \chapter*{Resumen}
    \@abstractsp

    \textbf{Palabras clave}: \@keywordssp
}%

\newcommand\makeabstract{
    \clearpage
    \thispagestyle{empty}
    \chapter*{Abstract}
    \@abstracten

    \textbf{Keywords}: \@keywordsen
}%
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Glosario
%
\RequirePackage[acronym,section,nowarn]{glossaries-extra}
% Hacemos que no haya salto de página entre glosario y siglas
\renewcommand*{\glsclearpage}{}
% Desactivamos de forma global la s del plural de los acrónimos
\setabbreviationstyle{long-short}
\renewcommand*{\glspluralsuffix}{}

\makeglossaries

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Cubierta trasera
%
\makeatletter         
\newcommand{\makebackcover}{
    \clearpage
    \thispagestyle{empty}
    \pagecolor{schoolcolor}
    \AddToShipoutPicture*{%
        \put(0,0){%
            \colorbox{black}{%
                \parbox[b][.5\paperheight]{\paperwidth}{\vfill\hfill}
            }%
        }
    }
    \vspace*{\fill}
    \begin{center}
        \includegraphics[width=.35\paperheight]{\schoollogoblackbackground}
    \end{center}
}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Elementos del comienzo del documento
%
\makeatletter
\AtBeginDocument{%
    \maketitle
    \makelicense
    \makereportquotation
    \makeacknowledgements
    \makeresumen
    \makeabstract
    
    \frontmatter

    \begingroup
        \tableofcontents
        \conditionalLoF
        \conditionalLoT
        \conditionalLoL
    \endgroup

    \mainmatter
}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Elementos del fin del documento
%
\makeatletter
\AtEndDocument{
    \backmatter

    \renewcommand\bibname{Referencias}
    \printbibliography
    \chapter*{Índice de términos}
    \printglossaries
    %\printindex  TODO Hacer el índice (por supuesto opcional)
    \makebackcover
}
\makeatother

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Añadimos el fichero con las entradas de las referencias bibliográficas
%
\addbibresource{references.bib}

\endinput