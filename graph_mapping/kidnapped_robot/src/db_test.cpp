#include <sqlite3.h>
#include <cstdlib>
#include <cstdio>
#include <stdexcept>

void checkErr(int returned_code, sqlite3* db, const char* msg, int expected_code = SQLITE_OK)
{
  if (returned_code != expected_code) {
    char full_msg[512];
    snprintf(full_msg, sizeof(full_msg), "%s: %s\n", msg, sqlite3_errmsg(db));
    throw std::runtime_error(full_msg);
  }
}

int main(int argc, char** argv)
{
  // Open database connection
  /// @todo Extra options of sqlite3_open_v2 useful?
  sqlite3 *db = NULL;
  checkErr(sqlite3_open("mytest.db", &db), db, "Couldn't open database");

  // Prepared statement for creating table
  const char CREATE_SQL[] =
    "CREATE TABLE IF NOT EXISTS places"
    "(id INTEGER PRIMARY KEY,"
    " stamp INTEGER,"
    " map_position_x DOUBLE, map_position_y DOUBLE, map_position_z DOUBLE,"
    //" map_orientation_x DOUBLE, map_orientation_y DOUBLE, map_orientation_z DOUBLE, map_orientation_w DOUBLE,"
    /// @todo Transform from camera frame to odom
    " keypoints BLOB, descriptors BLOB)";
  sqlite3_stmt *create_table_stmt;
  checkErr(sqlite3_prepare_v2(db, CREATE_SQL, sizeof(CREATE_SQL), &create_table_stmt, NULL),
           db, "Couldn't prepare CREATE TABLE statement");

  // Execute CREATE TABLE statement
  checkErr(sqlite3_step(create_table_stmt), db, "CREATE TABLE statement not done", SQLITE_DONE);

  // Finalize the CREATE TABLE statement
  checkErr(sqlite3_finalize(create_table_stmt), db, "Couldn't finalize CREATE TABLE statement");

  // Prepared statement for inserting some data
  const char INSERT_SQL[] = "INSERT INTO places VALUES (NULL, $stamp, $pos_x, $pos_y, $pos_z, $keypts, $descs)";
  sqlite3_stmt *insert_stmt;
  checkErr(sqlite3_prepare_v2(db, INSERT_SQL, sizeof(INSERT_SQL), &insert_stmt, NULL),
           db, "Couldn't prepare INSERT statement");

  // Bind values
  //checkErr(sqlite3_bind_int(insert_stmt, 1, 3), db, "Couldn't bind id");
  checkErr(sqlite3_bind_int64(insert_stmt, 1, (sqlite3_int64)567), db, "Couldn't bind stamp");
  checkErr(sqlite3_bind_double(insert_stmt, 2, 1.0), db, "Couldn't bind map_position_x");
  checkErr(sqlite3_bind_double(insert_stmt, 3, 2.0), db, "Couldn't bind map_position_y");
  checkErr(sqlite3_bind_double(insert_stmt, 4, 3.0), db, "Couldn't bind map_position_z");
  const char KEYPOINT_DATA[] = "Keypoint data goes in here";
  const char DESCRIPTOR_DATA[] = "Descriptor data goes in here";
  checkErr(sqlite3_bind_blob(insert_stmt, 5, KEYPOINT_DATA, sizeof(KEYPOINT_DATA), SQLITE_TRANSIENT),
           db, "Couldn't bind keypoints");
  checkErr(sqlite3_bind_blob(insert_stmt, 6, DESCRIPTOR_DATA, sizeof(DESCRIPTOR_DATA), SQLITE_TRANSIENT),
           db, "Couldn't bind descriptors");

  // Execute INSERT statement
  checkErr(sqlite3_step(insert_stmt), db, "INSERT statement not done", SQLITE_DONE);

  printf("Last insert rowid = %li\n", sqlite3_last_insert_rowid(db));
  
  /// @todo Reset INSERT statement
  
  // Finalize the insert statement
  checkErr(sqlite3_finalize(insert_stmt), db, "Couldn't finalize INSERT statement");
  
  // Close database connection
  checkErr(sqlite3_close(db), db, "Couldn't close database");

  return 0;
}
