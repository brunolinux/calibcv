
#pragma once

#include <MWindow.h>
#include <pthread.h>

using namespace std;

namespace mauro 
{


	class MWindowManager
	{

		private :

		unordered_map< string, MWindow* > m_windows;

		GLFWwindow* m_sharedWindow;

		pthread_t m_threadHandle;
		bool m_initialized;
		bool m_isActive;

		MWindowManager();

		public :

		static MWindowManager* INSTANCE;
		static MWindowManager* create();
		static void release();

		~MWindowManager();

		MWindow* createWindow( string windowName, int width, int height );

		void run();
		void stop();
		static void* renderCall( void* _ );
		
		GLFWwindow* getSharedWindow() { return m_sharedWindow; }
		unordered_map< string, MWindow* > getWindows() { return m_windows; }
		bool isActive() { return m_isActive; }
		bool isInitialized() { return m_initialized; }
	};




}