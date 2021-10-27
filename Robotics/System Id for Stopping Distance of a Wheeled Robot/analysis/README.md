# Preparation

## Prerequisites
- python3

## How to Install

```bash
$ python3 -m venv stop-start-robot-dev
$ source stop-start-robot-dev/bin/activate
$ pip install bokeh bagpy pandas jupyterlab
```

## Usage

### Prerequisites
* rosbags from the experiments [one-example](https://www.google.com)


* Copy the rosbags and try to remember the fullpath(/home/...) of data folder and data name you want to investigate (Unfortunately, multi-data analysis still on the way...)

* The command given below will kick-off the jupyter notebook  and open an html for further code visualisation.
```bash
$ source stop-start-robot-dev/bin/activate
$ jupyter-lab
```

* And time to enjoyyyy! 

I quite enjoyed the project, but it is such a vaguely defined definition of done which makes it really difficult to stop thinking about it. My approach is always, first infrastracture than boom boom!

## To export as html with embedded images

To be able to download a html with embedded images Nbextensions can be quite handy. The following commands do install the package and configure jupyter to enable it.

```bash
$ pip install jupyter_contrib_nbextensions
$ pip install jupyter_nbextensions_configurator
$ jupyter contrib nbextension install --user 
$ jupyter nbextensions_configurator enable --user
```
