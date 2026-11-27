function track = load_fs_track(id, json_file)
%LOAD_FS_TRACK  Load one of the website's tracks from data/tracks.json.
%
%   track = load_fs_track('fsuk_autocross') returns a struct with
%     x, y      (1xN) centre line, m
%     k         (1xN) path curvature = 1/R, 1/m (signed, left turn positive)
%     R         (1xN) curvature radius, m (Inf on straights)
%     ds        (1xN) segment lengths, m. ds(i) = distance from point i to i+1
%               (for an open track the last entry is unused)
%     closed    true for a circuit (lap starts where it ends)
%     standing_start, entry_speed, timed (skidpad), name, id
%
%   load_fs_track() with no input lists the available ids.
%   The thesis (section 4.2.1) works out R from three neighbouring points;
%   the website file already holds a smoothed curvature, so it is used as is.
%   For your own x/y data use track_from_xy.

if nargin < 2
    here = fileparts(mfilename('fullpath'));
    json_file = fullfile(here, '..', 'data', 'tracks.json');
end
data = jsondecode(fileread(json_file));
list = data.tracks;
if ~iscell(list)
    list = num2cell(list);
end
if nargin < 1 || isempty(id)
    for j = 1:numel(list)
        fprintf('%-16s %s\n', list{j}.id, list{j}.name);
    end
    track = [];
    return
end
t = [];
for j = 1:numel(list)
    if strcmp(list{j}.id, id)
        t = list{j};
    end
end
if isempty(t)
    error('load_fs_track:unknown', 'No track "%s". Call load_fs_track() for the list.', id);
end

track.id   = t.id;
track.name = t.name;
track.x = t.x(:)';  track.y = t.y(:)';  track.k = t.k(:)';
track.R = 1 ./ abs(track.k);
track.ds = t.ds * ones(size(track.k));
track.closed = logical(t.closed);
track.standing_start = logical(t.standing_start);
track.entry_speed = 10;
if isfield(t, 'entry_speed'), track.entry_speed = t.entry_speed; end
track.timed = [];
if isfield(t, 'timed'), track.timed = t.timed; end   % [start end] distances of timed laps, m
track.laps = 1;
if isfield(t, 'laps'), track.laps = t.laps; end
end
