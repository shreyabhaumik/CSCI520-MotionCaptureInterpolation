#include <iostream>
#include <fstream>
#include <string>
#include <sstream>

using namespace std;

int main(void) 
{
	string line, infname, outfname;
	int option;

    cout << "Type just the amc filename inside the mocapPlayer-starter folder without the foldername and extension - ";
	cin >> infname;
	cout << "Type:"<< endl;
	cout << "1 - for lfemur joint, rotation around X axis"<< endl;
	cout << "2 - for root joint, rotation around Z axis"<< endl;
	cin >> option;

	if (option == 1)
		outfname = "graph/" + infname + "-lfemurRotX.txt";
	else if (option == 2)
		outfname = "graph/" + infname + "-rootRotZ.txt";
	else
		return 0;

	ifstream infile;
	ofstream outfile;

	infile.open("mocapPlayer-starter/" + infname + ".amc");
	outfile.open(outfname);

	if(infile.is_open())
	{
		while(!infile.eof())
		{
			getline(infile,line);
			stringstream s(line);
    		string word;
  
    		int count = 0; 
    		while (s >> word)
    		{
				if ((option == 1) && (line.substr(0,6) == "lfemur") && (count == 1))
					outfile << word <<endl;
				if ((option == 2) && (line.substr(0,4) == "root") && (count == 6))
					outfile << word <<endl;
				count++;
			}
		}
		infile.close();
		outfile.close();
		cout << outfname.substr(6,(outfname.length()-5)) + " created in 'graph' folder"<< endl;
	}
	else
		cout << "Unable to open file" << endl;
	return 0;
}