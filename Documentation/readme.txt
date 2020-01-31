Doxygen Documentation Generator

Setting Up Doxygen:-
1.In directory structure, make a document folder in root directory then run the command doxygen -g to generate the dconfig file.


2.Edit the configuration file to fit the project. The config file contains detailed description of all the fields. However the following are important.
  INPUT:- Set the directories where doxygen should be looking for files.
  GENERATE HTML:- Set this to yes.
  PROJECT NAME:- Sets the project name

3.Comment the code in doxygen format.

If you place comments before functions or variables use 
/** comment line1 
* comment line2
* comment line3 
*/

If you place comments after functions or variables use 
/**< comment line1
* comment line2
* comment line3
 */

 @ or \ preceeded text are considered as doxygen commands

 Important commands are:-

 \class <name> [<header-file>] [<header-name>]
Ex-
 /*! \class Test class.h "inc/class.h"
 *  \brief This is a test class.
 *
 * Some details about the Test class.
 */
class ABC{};


\brief 
starts brief description


\details
starts detailed description

\param '['dir']' <parameter-name> { parameter description }
Starts a parameter description for a function parameter with name <parameter-name>, followed by a description of the parameter. 
The \param command has an optional attribute, dir, specifying the direction of the parameter. Possible values are "[in]", "[in,out]", and "[out]", note the [square] brackets in this description. When a parameter is both input and output, [in,out] is used as attribute.
Ex-
/*!
 * Copies bytes from a source memory area to a destination memory area,
 * where both areas may not overlap.
 * @param[out] dest The memory area to copy to.
 * @param[in]  src  The memory area to copy from.
 * @param[in]  n    The number of bytes to copy
 */
void memcpy(void *dest, const void *src, size_t n);

Link to official documentation of all the commands
http://www.doxygen.nl/manual/commands.html



