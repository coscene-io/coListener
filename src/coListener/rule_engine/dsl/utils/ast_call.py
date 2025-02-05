import ast
import sys


def create_ast_call(func, args, keywords, starargs = None, kwargs = None):
    if sys.version_info[0] == 2:
        return ast.Call(
            func=func,
            args=args,
            keywords=keywords,
            starargs=starargs,
            kwargs=kwargs
        )
    else:
        return ast.Call(func=func, args=args, keywords=keywords)