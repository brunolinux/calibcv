
#include <MWindowManager.h>


using namespace std;



int main()
{

	auto _wmanager = mauro::MWindowManager::create();
	auto _wTest = _wmanager->createWindow( "Test window", 400, 400 );

	_wmanager->run();

	while( 1 )
	{
		std::cout << "sleeping for 1 second :D" << std::endl;
		std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );
	}

	mauro::MWindowManager::release();

	return 0;
}