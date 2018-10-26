
__version__ = '0.1.3-alpha'


class UbidotsLibraryError (Exception):
    pass

def check_requests_version():
	minimum_requests_version = '1.2.3'
	try:
		import urequests
	except:
		raise UbidotsLibraryError("""requests Library is not installed, please install it
		with pip install requests or better yet install ubidots using pip install ubidots""")

	
check_requests_version()

from .apiclient import ApiClient, Datasource, Variable, ServerBridge, Paginator
