Racing lines for Silverstone and Hockenheim from the TUM racetrack database
(https://github.com/TUMFTM/racetrack-database), licensed LGPL-3.0. Files are unmodified apart from the file name.
Columns: x (m), y (m).

cones/: cone positions of real competition layouts, converted to one simple form
(left row, right row, big orange start cones, start position and heading; metres).
- fsg18.json, fsi18.json: cone maps FSG.yaml and FSI.yaml shipped with AMZ Racing's FSSIM
  (https://github.com/AMZ-Racing/fssim). MIT licence, copyright (c) 2019 Akademischer
  Motorsportverein Zuerich. FSSIM was used to predict lap times at FSG 2018, so FSG.yaml is taken
  to be that track. Reading FSI as FS Italy 2018 is an assumption.
- fsg19.json, fss19.json, fse22.json, fsg23.json: from the layout-merchant collection
  (https://github.com/papalotis/layout-merchant). MIT licence, copyright (c) 2024 Panagiotis.
  Traced from competition video by its author: cone positions relative to each other are right,
  the overall scale was estimated.
All six are driverless-class courses. Values are rounded to 0.01 m; nothing else was changed.
