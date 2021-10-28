#include <iostream>
#include <locale>
#include "mapper/costmap.h"

using namespace std;

int main() {
    double resolution;
    int rows, cols;

    char choice;

    cout << "ENTER A RESOLUTION" << endl;

    cin >> resolution;

    cout << "ENTER #ROWS" << endl;

    cin >> rows;

    cout << "ENTER #COLS" << endl;

    cin >> cols;


    CostMap map(resolution, 0, 0, rows, cols);

    do {
        cout << "ENTER ONE OF THE FOLLOWING:\n" <<
        "G: GET" << endl <<
        "L: LENGTH" << endl <<
        "P: PRINT" << endl <<
        "S: SET" << endl <<
        "0: QUIT" << std::endl;

        cin >> choice;

        switch(std::toupper(choice)) {
            case 'G': {
                int row, col;

                cout << "\tENTER A ROW\n\t";

                cin >> row;

                cout << "\n\tENTER A COLUMN\n\t";

                cin >> col;


                cout << "\tVALUE OF (" << row << ", " << col << "): " << map.get(row, col) << endl;

                break;
            }

            case 'L': {
                cout << "\tLENGTH: " << map.length() << endl;

                break;
            }

            case 'P': {
                cout << "\t" << map << endl;

                break;
            }

            case 'S': {
                int row, col;
                double value;

                cout << "\tENTER A ROW\n\t";

                cin >> row;

                cout << "\n\tENTER A COLUMN\n\t";

                cin >> col;

                cout << "\n\tENTER A VALUE\n\t";

                cin >> value;

                map.set(row, col, value);

                break;
            }

            case '0': break;

            default: 
                cout << "INVALID CASE" << endl;
        }

        cout << endl;

    } while(choice != '0');

    cout << "PROGRAM TERMINATED" << endl;

    return 0;
}
