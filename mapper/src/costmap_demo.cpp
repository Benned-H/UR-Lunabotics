#include <iostream>
#include <locale>
#include "mapper/costmap.h"

using namespace std;

int main( int argc, char* argv[] ){

    double resolution;
    int rows, cols;

    char choice;

    std::cout << "ENTER A RESOLUTION" << std::endl;

    cin >> resolution;

    cout << "ENTER #ROWS" << endl;

    cin >> rows;

    cout << "ENTER #COLS" << endl;

    cin >> cols;


    CostMap map(resolution, 0, 0, rows, cols);

    do {
        cout << "ENTER ONE OF THE FOLLOWING:\n" <<
        "G: GET" << endl <<
        "I: POINT_TO_INDEX" << endl <<
        "L: LENGTH" << endl <<
        "M: IN_MAP" << endl <<
        "P: PRINT" << endl <<
        "S: SET" << endl <<
        "X: X_TO_COL" << endl <<
        "Y: Y_TO_ROW" << endl <<
        "0: QUIT" << std::endl;

        cin >> choice;

        switch(std::toupper(choice)) {
            case 'G': {
                double x, y;

                cout << "\tENTER A VALUE FOR X\n\t";

                cin >> x;

                cout << "\n\tENTER A VALUE FOR Y\n\t";

                cin >> y;

                cout << "\tVALUE OF (" << x << ", " << y << "): " << map.get(x, y) << endl;

                break;
            }

            case 'I': {
                double x, y;

                cout << "\tENTER A VALUE FOR X\n\t";

                cin >> x;

                cout << "\n\tENTER A VALUE FOR Y\n\t";

                cin >> y;

                cout << "\n\tINDEX OF (" << x << ", " << y << "): " << map.point_to_index(x, y) << endl;

                break;
            }

            case 'L': {
                cout << "\tLENGTH: " << map.length() << endl;

                break;
            }

            case 'M': {
                double x, y;

                cout << "\tENTER A VALUE FOR X\n\t";

                cin >> x;

                cout << "\n\tENTER A VALUE FOR Y\n\t";

                cin >> y;

                cout << "\n\t(" << x << ", " << y << ") IS " << (!map.in_map(x, y) ? "NOT " : "") << "IN THE MAP" << endl;

                break;
            }

            case 'P': {
                cout << map << endl;

                break;
            }

            case 'S': {
                double x, y;
                double value;

                cout << "\tENTER A VALUE FOR X\n\t";

                cin >> x;

                cout << "\n\tENTER A VALUE FOR Y\n\t";

                cin >> y;

                cout << "\n\tENTER A VALUE\n\t";

                cin >> value;

                map.set(x, y, value);

                break;
            }

            case 'X': {
                double x;

                cout << "\n\tENTER A VALUE FOR X\n\t";

                cin >> x;

                cout << "\tX TO COL: " << map.x_to_col(x) << endl;

                break;
            }

            case 'Y': {
                double y;

                cout << "\n\tENTER A VALUE FOR Y\n\t";

                cin >> y;

                cout << "\tX TO ROW: " << map.y_to_row(y) << endl;

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
