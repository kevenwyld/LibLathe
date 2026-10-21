
# Contribution Guidelines: 

## Coding Conventions

Code to follow PEP8 style guide https://www.python.org/dev/peps/pep-0008

### Naming conventions:

Type | Style
------------ | -------------
class Names | CappedWords
module, function and variables | under_scored
private functions and variables | _leading_underscore 
constants | UPPERCASE 

### Testing
All additional functionalilty should be complete with relevant tests where possible. Please ask if you are unsure.

Run the tests from the repository root:

    python3 -m unittest discover -s tests

`tests/test_regression.py` runs every operation on the parts in
`tests/data` and compares the G-code with `tests/golden`: a change that
alters a toolpath fails there until it is checked. If the new output is
intended, regenerate the golden files and commit them with the change:

    LIBLATHE_UPDATE_GOLDEN=1 python3 -m unittest discover -s tests

`tests/test_behaviour.py` checks specific results the operations must keep
(e.g. a negative stock to leave turning a 12 mm bar to 11.85, Partoff going
past a bore, the thread's root and clearance checks).
