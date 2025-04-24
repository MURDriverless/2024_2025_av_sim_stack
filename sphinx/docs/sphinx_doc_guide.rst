Sphinx Documentation Guide
==========================

Sphinx is an auto documentation generator that makes building a documentation/wiki site a breeze. The language used to make these "pages" is known as **reStructuredText (reST)**, which is very similar to markdown. The main difference is that reST is a bit more verbose, where the same features in markdown is limited (e.g. tables).

Sphinx takes these plain text files written in reStructuredText (or Markdown, with plugins) and turns them into well-formatted documents like HTML pages, PDF manuals, and EPUB books. In this case, it's HTML pages. 

To setup up Sphinx on your machine, please follow the instructions given in the `official installation guide <https://www.sphinx-doc.org/en/master/usage/installation.html#>`_.

.. contents:: Content
   :local:
   :depth: 2
   :backlinks: entry

Directory structure
-------------------

The typical directory structure of Sphinx is:

.. code-block:: text

   docs/
   ├── _build/             # Compiled output (HTML, PDF, etc.)
   ├── _static/            # Custom static files (e.g., CSS, images)
   ├── _templates/         # Custom HTML templates
   ├── conf.py             # Sphinx configuration file
   ├── index.rst           # Root document
   ├── example_page1.rst   # Example page 1
   ├── example_page2.rst   # Example page 2
   └── other_pages.rst     # Additional docs

.. note::

    This only applied if you have not separated the source and build directory during:
    
    .. code-block:: text
        
        sphinx-quickstart

If the source and build directories were separated during quickstart, then the directory structure should look like:

.. code-block:: text

   docs/
   ├── build/                  # Compiled output (HTML, PDF, etc.)
   ├── source/
   │   ├── _static/            # Custom static files (e.g., CSS, images)
   │   ├── _templates/         # Custom HTML templates
   │   ├── conf.py             # Configuration file for Sphinx
   │   ├── index.rst           # Root document
   │   ├── example_page1.rst   # Example page 1
   │   ├── example_page2.rst   # Example page 2
   │   └── other_pages.rst     # Other documentation pages
   ├── Makefile                # For Unix-based make builds
   └── make.bat                # For Windows builds

For our Sphinx setup, it is the former where the the directories are **NOT** separated.

Common Sphinx Commands
----------------------

When working in the root directory, *i.e ./docs/*, and assuming you've installed sphinx, you can cleanly build the HTML pages using:

.. code-block:: text

    make clean       # Removes and cleans the _build/ folder. This ensures that pages a refreshed with the changes you've made
    make html        # Builds the HTML pages

This is possible due to the Makefile and make.bat files in the root directory which were created during ``sphinx-quickstart``

Editing Pages
-------------
The contents of the main page of the documentation is held within ``./docs/index.rst``. Any edits to the main page should be conducted within that file.

As stated before, the syntax in Sphinx is reStructuredText so you'll need to familiarize yourself with the language to properly edit the documentation. To see documentation for reStructuredText, see the following `link <https://www.sphinx-doc.org/en/master/usage/restructuredtext/index.html>`_.