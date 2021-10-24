# Preparation

## Prerequisites
- python3

```bash
$ python3 -m venv stop-start-robot-dev
$ source stop-start-robot-dev/bin/activate
$ pip install bokeh bagpy pandas jupyterlab
```

## Usage

The command given below will kick-off the jupyter notebook 

and open an html for further code visualisation.

```bash
$ jupyter-lab
```

## To export as html with embedded images

To be able to download a html with embedded images Nbextensions can be quite handy. The following commands do install the package and configure jupyter to enable it.

```bash
$ pip install jupyter_contrib_nbextensions
$ pip install jupyter_nbextensions_configurator
$ jupyter contrib nbextension install --user 
$ jupyter nbextensions_configurator enable --user
```
