#include "mysql_connection.h"
#include <cppconn/driver.h>
#include <cppconn/exception.h>
#include <cppconn/resultset.h>
#include <cppconn/statement.h>
#include <cppconn/prepared_statement.h>

using namespace std;

int main()
{
 try {
  sql::Driver *driver;
  sql::Connection *con;
  sql::Statement *stmt;
  sql::ResultSet *res;

  /* Create a connection */
  driver = get_driver_instance();
  con = driver->connect("tcp://127.0.0.1:3306", "malcomneo", "malcomneo");
  /* Connect to the MySQL test database */
  con->setSchema("inventory");

  stmt = con->createStatement();
  stmt->execute("DELETE FROM workcell");

  stmt->execute("INSERT INTO workcell VALUES (1,'basin'),(2,'basin'),(3,'instrument1'),(4,'instrument2'),(5,'instrument1'),(6,'instrument5'),(7,'basin')");

 }

catch (sql::SQLException &e) {
  cout << "# ERR: SQLException in " << __FILE__;
  cout << "(" << __FUNCTION__ << ") on line "      << __LINE__ << endl;
  cout << "# ERR: " << e.what();
  cout << " (MySQL error code: " << e.getErrorCode();
  cout << ", SQLState: " << e.getSQLState() << " )" << endl;
}

return EXIT_SUCCESS;
}