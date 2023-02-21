#include "Pathloader.hpp"

AutonPath lastpath;

AutonPath PathLoader(std::string PathName)
{
    AutonPath path;
    
    // Check if our path was recently loaded
    if (lastpath.Name == PathName)  
    {
        // Get our path location
        fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
        deployDirectory = deployDirectory / "paths" / PathName / ".wpilib.json";

        std::fstream ostream;
        // Open our file
        ostream.open(deployDirectory);

        // Find and record the total lenght of the file
        ostream.seekg(0, std::ios::end);
        std::streampos lenght = ostream.tellg();
        ostream.seekg(0, std::ios::beg);

        // Store our char steam into a char buffer
        std::vector<char> buffer(lenght);
        ostream.read(&buffer[0], lenght);

        std::string out(buffer.begin(), buffer.end());

        ostream.close(); // Close our file stream

        nlohmann::json jsonOut = nlohmann::json::parse(out);

        for (int i = 0; i < jsonOut.size(); i++)
        {
            path.time.push_back((double)jsonOut[i]["time"]);
            path.rot.push_back((double)jsonOut[i]["pose"]["rotation"]["radians"]);
            path.x.push_back((double)jsonOut[i]["pose"]["translation"]["x"]);
            path.y.push_back((double)jsonOut[i]["pose"]["translation"]["y"]);
        }
        path.Name = PathName;

        lastpath = path; // Store our current path

        return path;
    } else {
        return lastpath;
    }
}
