CREATE TABLE IF NOT EXISTS patrol (
  `id` INT PRIMARY KEY AUTO_INCREMENT,
  `active` INT DEFAULT 0,
  `circuit` JSON,
  `start` TIMESTAMP,
  `end` TIMESTAMP
);

CREATE TABLE IF NOT EXISTS inspection (
  `uid` VARCHAR(255) PRIMARY KEY,
  `col` INT NOT NULL,
  `row` INT NOT NULL,
  `status` VARCHAR(255)
);

CREATE TABLE IF NOT EXISTS cover_dispatch (
  `uid` VARCHAR(255),
  `provider` VARCHAR(255) NOT NULL,
  `accepted` INT DEFAULT -1,
  INDEX(`uid`)
);
