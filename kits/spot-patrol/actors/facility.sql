CREATE TABLE IF NOT EXISTS inspection_dispatch (
  `uid` VARCHAR(255),
  `provider` VARCHAR(255) NOT NULL,
  `distance` REAL,
  `accepted` INT DEFAULT -1,
  INDEX(`uid`)
);

CREATE OR REPLACE VIEW priority_inspection_dispatch AS
  SELECT * FROM inspection_dispatch WHERE distance IS NOT NULL ORDER BY distance ASC;