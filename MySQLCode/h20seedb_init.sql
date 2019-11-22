# H2Osee.com
# database initialization script
#
# Mark Pacetti
#Sep 1, 2019
#
# used to create h2oseedb database and sample table
#
# Oct 1, 2019
#
# =======================================================================================

-- drop database and recreate
/* DROP DATABASE h2oseedb;

 CREATE DATABASE h2oseedb
	CHARACTER SET `utf8`
    COLLATE `utf8_general_ci`;*/

USE h2oseedb;

-- drop tables to rebuild tables

DROP TABLE h2oseedb.`sample`;

CREATE TABLE h2oseedb.sample
(
   sampleid      INT(20) UNSIGNED NOT NULL AUTO_INCREMENT,
   dbtimestamp   TIMESTAMP(0) NOT NULL DEFAULT CURRENT_TIMESTAMP,
   topicbase     VARCHAR(25) CHARACTER SET utf8 COLLATE utf8_general_ci NOT NULL,
   locationid    VARCHAR(10) CHARACTER SET utf8 COLLATE utf8_general_ci NOT NULL,
   sensor        VARCHAR(35) CHARACTER SET utf8 COLLATE utf8_general_ci NOT NULL,
   reading       VARCHAR(70) CHARACTER SET utf8 COLLATE utf8_general_ci NOT NULL,
   PRIMARY KEY(sampleid)
)
ENGINE INNODB
COLLATE 'utf8_general_ci'
ROW_FORMAT DEFAULT;

-- check that table structure was created
select COUNT(*) from h2oseedb.`sample`;
select * from sample;
-- ===================================================================================