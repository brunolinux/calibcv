
#include <MWindowManager.h>


using namespace std;



int main()
{

	auto _wmanager = mauro::MWindowManager::create();
	auto _wTest1 = _wmanager->createWindow( "Test window 1", 400, 400 );
	auto _wTest2 = _wmanager->createWindow( "Test window 2", 600, 600 );

	_wmanager->run();

	while( 1 )
	{
		std::cout << "sleeping for 1 second :D" << std::endl;
		std::this_thread::sleep_for( std::chrono::milliseconds( 1000 ) );

		// _wTest1->plotQuadPattern( rot, trans, dims );
	}

	_wmanager->stop();

	mauro::MWindowManager::release();

	return 0;
}