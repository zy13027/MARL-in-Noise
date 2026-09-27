% Unit tests for the grid-world environments and message helpers.
% Run from the project root with:
%   setupProject
%   results = runtests("tests")

%% act1Conv matches the original lookup table
m = act1Conv((1:8)');
assert(isequal(m, [0 0 0 0; 0 0 0 1; 0 0 1 0; 0 0 1 1; 0 1 0 0; 0 1 0 1; 0 1 1 0; 0 1 1 1]))
assert(isequal(act1Conv(6), [0 1 0 1]))

%% act1Conv rejects actions outside 1 to 8
for a = [0 9 2.5]
    id = '';
    try
        act1Conv(a);
    catch err
        id = err.identifier;
    end
    assert(strcmp(id,'act1Conv:outOfRange'))
end

%% deconvert inverts act1Conv
m = act1Conv((1:8)');
assert(isequal(deconvert(m), (1:8)'))
assert(deconvert(m(3,:).') == 3)                    % 4x1 Hamming Decoder frame
assert(isequal(deconvert(reshape(m.',[],1)), (1:8)'))  % bit stream
assert(deconvert([1 1 0 1]) == 6)                   % padding bit ignored

%% simouttodec returns the most frequent message
bits = act1Conv([3 3 5 3 8 3]');
assert(simouttodec(reshape(bits.',4,1,[])) == 3)    % 4x1xN To Workspace data
assert(simouttodec(bits) == 3)                      % Nx4
assert(simouttodec(bits.') == 3)                    % 4xN
assert(simouttodec(bits(5,:).') == 8)               % single frame

%% GridWorld single robot: sizes and a diagonal move
g = GridWorld('InitialStates',[1 1]);
[o,r,d] = step(g,5);
assert(isequal(size(o),[64 64 4]) && isequal(size(r),[1 1]))
assert(isequal(g.States,[2 2]) && abs(r - 0.95) < 1e-12 && d == 0)
assert(o(2,2,2) == 1 && nnz(o(:,:,2)) == 1)          % self channel
assert(nnz(o(:,:,3)) == 0)                          % no friends
assert(nnz(o(:,:,4)) == 64*64 - 2)                  % unexplored cells

%% GridWorld blocks every move off the grid
g = GridWorld('InitialStates',[1 1]);
for a = [2 3 6 7 8]
    [~,r] = step(g,a);
    assert(isequal(g.States,[1 1]) && r == -0.5)
end
g = GridWorld('InitialStates',[64 64]);
for a = [1 4 5 6 7]
    [~,r] = step(g,a);
    assert(isequal(g.States,[64 64]) && r == -0.5)
end
[~,r] = step(g,8);
assert(isequal(g.States,[63 63]) && abs(r - 0.95) < 1e-12)

%% GridWorld wait and revisit rewards
g = GridWorld('InitialStates',[5 5]);
[~,r] = step(g,0);
assert(r == -0.5 && isequal(g.States,[5 5]))
step(g,1);
[~,r] = step(g,2);
assert(r == -1/20)

%% GridWorld obstacles
g = GridWorld('InitialStates',[5 5],'Obstacles',[6 5; 6 6]);
[~,r] = step(g,1);
assert(isequal(g.States,[5 5]) && r == -0.5)
[o,r] = step(g,5);
assert(isequal(g.States,[5 5]) && r == -0.5)
assert(nnz(o(:,:,1)) == 2 && o(6,5,1) == 1)

%% GridWorld full coverage ends the episode
g = GridWorld('InitialStates',[1 1],'GridSize',[3 3]);
for a = [4 4 1 3 3 1 4 4]
    [~,r,d] = step(g,a);
end
assert(d == 1 && abs(r - (19 + 4000)/20) < 1e-9)

%% GridWorld three robots: collisions and friend channel
g = GridWorld('InitialStates',[2 2; 3 2; 2 4],'GridSize',[12 12],'NumRobots',3);
[o,r] = step(g,[1;0;3]);
assert(isequal(size(o),[12 12 4 3]) && isequal(size(r),[3 1]))
assert(isequal(g.States,[2 2; 3 2; 2 3]) && isequal(r,[-0.5; -0.5; 0.95]))
assert(o(3,2,3,1) == 1 && o(2,3,3,1) == 1 && nnz(o(:,:,3,1)) == 2)

%% GridWorld validates its properties
id = '';
try
    step(GridWorld('NumRobots',4),[1;1;1;1]);
catch err
    id = err.identifier;
end
assert(strcmp(id,'GridWorld:NumRobots'))
id = '';
try
    step(GridWorld('InitialStates',[70 1]),1);
catch err
    id = err.identifier;
end
assert(strcmp(id,'GridWorld:InitialStates'))

%% CustomGridWorld terminal state
c = CustomGridWorld('InitialStates',[1 1],'TerminalStates',[2 2]);
[~,r,d] = step(c,4);
assert(d == 0 && abs(r - 0.95) < 1e-12)
[~,r,d] = step(c,1);
assert(d == 1 && abs(r - 1.45) < 1e-12)

%% GridWorld matches the original MathWorks example (GridWorldV0)
% 12x12, three robots, actions 0 to 4, until full coverage
rng(1);
for ep = 1:10
    s0 = [2 2; 11 4; 3 12];
    old = GridWorldV0('InitialStates',s0,'MaxStepCount',0);  % 0 = no plots
    new = GridWorld('InitialStates',s0,'GridSize',[12 12],'NumRobots',3);
    for t = 1:400
        a = randi([0 4],3,1);
        [o1,r1,d1] = step(old,a);
        [o2,r2,d2] = step(new,a);
        assert(isequal(o1,o2) && d1 == d2 && isequal(old.States,new.States))
        if d1
            break  % full-coverage bonus is split per robot in GridWorld
        end
        assert(isequal(r1,r2))
    end
end
