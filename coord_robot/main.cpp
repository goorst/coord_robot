#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <string>

using namespace std;

// структора маячков
struct Beacon 
{
    double x;
    double y;
    double z;
    double distance;
};

// функция для чтения файла
vector<Beacon> read_file(const string& filename, size_t& count_beacons, bool& flag)
{
    vector<Beacon> beacons;
    ifstream file(filename);

    if (!file.is_open())
    {
        cout << "File opening error: " << filename << endl;
        flag = false;
        return beacons;
    }

    flag = true;

    double x, y, z, distance;

    while (file >> x >> y >> z >> distance)
    {
        beacons.push_back({ x, y, z, distance });
        count_beacons++;
    }

    cout << "Total beacons in the file: " << count_beacons << "\n\n";

    file.close();

    return beacons;
}

// функция для проверки данных в файле 
bool is_valid_beacon_data(const string& filename) 
{
    ifstream file(filename);

    if (!file.is_open()) 
    {
        return false; 
    }

    string line;

    while (getline(file, line)) 
    {
        istringstream iss(line);
        double value;
        int count = 0;

        while (iss >> value) 
        {
            ++count;
        }

        if (count != 4 || !iss.eof()) 
        {
            file.close();
            return false; 
        }
    }

    file.close();

    return true;
}

// функция для поиска координатов робота
Beacon find_robot_coord(const vector<Beacon>& beacons, bool flag)
{
    if (flag)
    {
        if (beacons.size() < 3)
        {
            cout << "You need at least 3 beacons to find the coordinates of the robot.\n\n";
            return { 0, 0, 0, 0 };
        }
    }

    vector<vector<double>> A(beacons.size(), vector<double>(3));
    vector<double> b(beacons.size());

    for (size_t i = 0; i < beacons.size(); ++i)
    {
        A[i][0] = 2 * (beacons[i].x - beacons[i].distance);
        A[i][1] = 2 * (beacons[i].y - beacons[i].distance);
        A[i][2] = 2 * (beacons[i].z - beacons[i].distance);
        b[i] = pow(beacons[i].x, 2) + pow(beacons[i].y, 2) + pow(beacons[i].z, 2) - pow(beacons[i].distance, 2);
    }

    Eigen::MatrixXd matrixA(A.size(), A[0].size());
    Eigen::VectorXd vectorB(b.size());

    for (size_t i = 0; i < A.size(); ++i) 
    {
        for (size_t j = 0; j < A[0].size(); ++j) 
        {
            matrixA(i, j) = A[i][j];
        }

        vectorB(i) = b[i];
    }

    Eigen::VectorXd res = matrixA.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(vectorB);

    return { res(0), res(1), res(2), 0 };
}

// функция main
int main()
{
    char povt;
    cout << "\t\tMENU\n\nHow to use the program:\n1)Enter the full (absolute) path to the file (Example D:/directory/file.txt ).\n\n";

    do
    {
        string filename;
        cout << "Enter the absolute path to the file: ";
        cin >> filename;
        cout << endl;

        size_t count_beacons = 0;
        bool flag;

        vector<Beacon> beacons = read_file(filename, count_beacons, flag);

        count_beacons = 0;

        if (flag)
        {
            if (!is_valid_beacon_data(filename))
            {
                cout << "Incorrect data in the file. Check the format: x y z (coordinates) and distance (distance to the robot).\n\n";
            }
            else
            {
                Beacon robot_coordinates = find_robot_coord(beacons, flag);
                cout << "Coordinates of the robot: (" << robot_coordinates.x << ", " << robot_coordinates.y << ", " << robot_coordinates.z << ")\n\n";
            }
        }


        cout << "Repeat it?(Y/n): ";
        cin >> povt;
        cout << endl;

    } while (povt == 'y' || povt == 'Y');

    cout << "Press any button..." << endl;
    cin.ignore();
    cin.get();
    
    return 0;
}


// D:\project_in_C\qwe.txt
// D:\project_in_C\Doma_1\10test.txt
// D:\project_in_C\Doma_1\50test.txt
// D:\project_in_C\Doma_1\100test.txt
