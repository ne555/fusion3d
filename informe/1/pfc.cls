%vim: set filetype=tex:

\ProvidesClass{pfc}
\LoadClass[a4paper]{report}
\RequirePackage{polyglossia} %reemplaza a babel
\setmainlanguage{spanish}

\RequirePackage{fontspec} %utf8
\RequirePackage[shortlabels]{enumitem}
%\setitemize{noitemsep,topsep=0pt,parsep=0pt,partopsep=0pt}
\setlist{noitemsep,nolistsep,topsep=0pt,parsep=0pt,partopsep=0pt}

\RequirePackage{pdfpages}
\RequirePackage{graphicx}
\RequirePackage[export]{adjustbox}
\RequirePackage{hyperref}

\RequirePackage{titlesec}
\titleformat{\chapter}
	{}
	{}
	{0pt}
	{\centering\bfseries\LARGE}
\renewcommand{\thesection}{\arabic{section}}
\renewcommand{\thesubsection}{\thesection.\arabic{subsection}}

\RequirePackage{caratula}

\Encabezado{Proyecto Final de Carrera\\
Ingeniería en Informática\\
\emph{Universidad Nacional del Litoral}}
\Logo{unl}

\subtitle{Reconstrucción de volúmenes para impresión 3D mediante fusión y rellenado de mallas de superficies parciales}
\author{Walter Bedrij}
\Director{Francisco Mainero}
\Codirector{Pablo Novara}
\date{\today}

\widowpenalty 10000
\clubpenalty 10000
