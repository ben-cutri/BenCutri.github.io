## A standardized guide on installing the required python libraries

For managing pip packages I strongly recommend creating and using a virtual environment. Think of it as a container for all your needed pip packages for snowBot (compared to globally installing pip packages). Command line arguments given are for Windows.

Create a virtual environment from the root of the project directory,

`python -m venv .env`

Activate it,

`.\.env\Scripts\activate`

Install all required pip packages into your virtual environment,

`pip install -r .\requirements.txt`

Couldn't find a way to install RPi.GPIO through pip so you have to install it some other way :(
