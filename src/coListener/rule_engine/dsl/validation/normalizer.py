import ast

from ..utils.ast_call import create_ast_call


# from ..ast import ast


class FStringToFormatTransformer(ast.NodeTransformer):
    def visit_JoinedStr(self, node):
        format_string = ""
        format_values = []

        for value in node.values:
            if isinstance(value, ast.Str):
                format_string += value.s
            elif isinstance(value, ast.FormattedValue):
                format_string += "{}"
                format_values.append(value.value)

        return create_ast_call(
            ast.Attribute(
                value=ast.Str(s=format_string),
                attr="format",
                ctx=ast.Load()
            ),
            format_values,
            []
        )
        # return ast.Call(
        #     func=ast.Attribute(
        #         value=ast.Str(s=format_string),
        #         attr="format",
        #         ctx=ast.Load()
        #     ),
        #     args=format_values,
        #     keywords=[],
        #     starargs=None,
        #     kwargs=None
        # )


def normalize_expression_tree(tree):
    return ast.fix_missing_locations(BooleanTransformer().visit(tree))


class BooleanTransformer(ast.NodeTransformer):
    def visit_JoinedStr(self, node):
        new_values = []
        condition_args = []
        for v in node.values:
            if isinstance(v, ast.Constant):
                new_values.append(v)
            elif isinstance(v, ast.FormattedValue):
                if v.format_spec:
                    # In theory, f-strings format spec can be another
                    # f-string, but it makes our logic very clumsy, so we
                    # just don't support it for now. Check here that it's
                    # only a constant.
                    if len(v.format_spec.values) != 1 or not isinstance(
                            v.format_spec.values[0], ast.Constant
                    ):
                        raise Exception("Formatting does not support nested values")

                new_values.append(
                    ast.FormattedValue(
                        ast.Name("arg{}".format(len(condition_args)), ast.Load()),
                        v.conversion,
                        v.format_spec,
                    )
                )
                condition_args.append(self.generic_visit(v.value))
            else:
                raise Exception("Shouldn't get here: {}".format(v))

        lambda_node = self._eval_expr(
            "lambda {0}: object()".format(",".join("arg" + str(i) for i in range(len(condition_args)))))
        lambda_node.body = ast.JoinedStr(new_values)
        return create_ast_call(
            ast.Name("func_apply", ast.Load()),
            [lambda_node] + condition_args,
            [],
        )
        # return ast.Call(
        #     func=ast.Name("func_apply", ast.Load()),
        #     args = [lambda_node] + condition_args,
        #     keywords = [],
        #     starargs=None,
        #     kwargs=None
        # )

    def visit_BoolOp(self, node):
        node = self.generic_visit(node)
        if isinstance(node.op, ast.And):
            func_name = "and_"
        elif isinstance(node.op, ast.Or):
            func_name = "or_"
        else:
            return node

        return create_ast_call(ast.Name(func_name, ast.Load()),node.values, [])
        # return ast.Call(
        #     func=ast.Name(func_name, ast.Load()),
        #     args=node.values,
        #     keywords=[],
        #     starargs=None,
        #     kwargs=None
        # )

    def visit_UnaryOp(self, node):
        node = self.generic_visit(node)
        if isinstance(node.op, ast.Not):
            return create_ast_call(ast.Name("not_", ast.Load()), [node.operand], [])
            # return ast.Call(
            #     func=ast.Name("not_", ast.Load()),
            #     args=[node.operand],
            #     keywords=[],
            #     starargs=None,
            #     kwargs=None
            # )
        else:
            return node

    def visit_Call(self, node):
        # A hack since 1. conditions have states, and 2. within SequenceMatchCondition,
        # we need to create clean copies of the condition objects to avoid state
        # sharing. As a result, we need to transform the condition arguments
        # of the SequenceMatchCondition related functions to condition factories,
        # with which we can lazy create copies of the condition objects
        # within the SequenceMatchCondition.
        node = self.generic_visit(node)
        if isinstance(node.func, ast.Name) and node.func.id in ("repeated", "debounce"):
            factory = self._eval_expr("lambda: object")
            factory.body = node.args[0]
            node.args[0] = factory
        elif isinstance(node.func, ast.Name) and node.func.id in (
                "sequential",
                "timeout",
        ):
            new_args = []
            for arg in node.args:
                factory = self._eval_expr("lambda: object()")
                factory.body = arg
                new_args.append(factory)
            node.args = new_args
        return node

    def visit_Compare(self, node):
        # We need to jump through quite a few hoops to keep inline with Python's
        # semantics for comparison operators, just so we can rewrite the `in`
        # operator into our `has` function call. Suppose you have the expression
        #
        #  a op1 b op2 c
        #
        # Python treats this as
        #
        #  a op1 b and b op2 c
        #
        # Except `b` must only be evaluated once. So we package the above into
        # the following:
        #
        # (lambda arg0, arg1, arg2:
        #   and_(
        #     arg0 op1 arg1,
        #     arg1 op2 arg2))(a, b, c)
        #
        # And if one of the operators happen to be an `in` operator, we swap it
        # out with a `has` call.

        node = self.generic_visit(node)
        args = [node.left] + node.comparators
        param_list = ["arg" + str(i) for i in range(len(args))]
        condition_list = []
        for i, (left, right) in enumerate(zip(param_list, param_list[1:])):
            if isinstance(node.ops[i], ast.In):
                condition_list.append(self._eval_expr("has({}, {})".format(right, left)))
            elif isinstance(node.ops[i], ast.NotIn):
                condition_list.append(self._eval_expr("not_(has({}, {}))".format(right, left)))
            else:
                condition_list.append(
                    ast.Compare(
                        ast.Name(left, ast.Load()),
                        [node.ops[i]],
                        [ast.Name(right, ast.Load())],
                    )
                )
        wrapper = self._eval_expr("lambda {}: object()".format(",".join(param_list)))
        wrapper.body = create_ast_call(ast.Name("and_", ast.Load()), condition_list, [])
        # wrapper.body = ast.Call(
        #     func=ast.Name("and_", ast.Load()),
        #     args=condition_list,
        #     keywords=[],
        #     starargs=None,
        #     kwargs=None
        # )
        # return ast.Call(
        #     func=wrapper,
        #     args=args,
        #     keywords=[],
        #     starargs=None,
        #     kwargs=None)
        return create_ast_call(wrapper, args, [])

    def _eval_expr(self, str_value):
        return ast.parse(str_value, mode="eval").body
