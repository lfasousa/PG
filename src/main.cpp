//General dependencies
#include "Classe.Projeto.h"

int main(int argc, char* argv[])
{
	//0 - Grabber 
	//11 - Player
	//12 - Player
	//13 - Player
	//2 - Recorder
	//3 - Render

	int p = 0;
	if (argc > 1)
	{
		std::stringstream sa(argv[1]);
		sa >> p ? p : 0;
	}
	TProjeto pg;
	pg.Run(p);

	return 0;
}
